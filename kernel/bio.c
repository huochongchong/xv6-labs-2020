// Buffer cache.
//
// The buffer cache is a linked list of buf structures holding
// cached copies of disk block contents.  Caching disk blocks
// in memory reduces the number of disk reads and also provides
// a synchronization point for disk blocks used by multiple processes.
//
// Interface:
// * To get a buffer for a particular disk block, call bread.
// * After changing buffer data, call bwrite to write it to disk.
// * When done with the buffer, call brelse.
// * Do not use the buffer after calling brelse.
// * Only one process at a time can use a buffer,
//     so do not keep them longer than necessary.

#include "types.h"
#include "param.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "riscv.h"
#include "defs.h"
#include "fs.h"
#include "buf.h"

#define NBUCKET 13
#define HASH(id) (id % NBUCKET)

struct hashbucket {
  struct spinlock lock;
  struct buf head;  // 链表的头节点
};

struct {
  struct buf buf[NBUF];
  struct hashbucket buckets[NBUCKET];
} bcache;

void
binit(void)
{
  struct buf *b;
  char lockname[16];

  // 初始化所有哈希桶
  for(int i = 0; i < NBUCKET; i++) {
    snprintf(lockname, sizeof(lockname), "bcache.bucket%d", i);
    initlock(&bcache.buckets[i].lock, lockname);
    
    // 创建空链表
    bcache.buckets[i].head.prev = &bcache.buckets[i].head;
    bcache.buckets[i].head.next = &bcache.buckets[i].head;
  }

  // 将所有缓冲区分配到第一个桶
  for(b = bcache.buf; b < bcache.buf + NBUF; b++) {
    // 将缓冲区插入到桶0的链表头部
    b->next = bcache.buckets[0].head.next;
    b->prev = &bcache.buckets[0].head;
    initsleeplock(&b->lock, "buffer");
    b->refcnt = 0;
    bcache.buckets[0].head.next->prev = b;
    bcache.buckets[0].head.next = b;
  }
}

// Look through buffer cache for block on device dev.
// If not found, allocate a buffer.
// In either case, return locked buffer.
static struct buf*
bget(uint dev, uint blockno)
{
  struct buf *b;
  int bid = HASH(blockno);
  
  acquire(&bcache.buckets[bid].lock);

  // 在当前桶中查找是否已缓存
  for(b = bcache.buckets[bid].head.next; b != &bcache.buckets[bid].head; b = b->next) {
    if(b->dev == dev && b->blockno == blockno) {
      b->refcnt++;
      b->timestamp = ticks;
      release(&bcache.buckets[bid].lock);
      acquiresleep(&b->lock);
      return b;
    }
  }

  // 没有找到缓存，需要寻找可重用的缓冲区
  // 首先在当前桶中查找
  struct buf *victim = 0;
  
  for(b = bcache.buckets[bid].head.next; b != &bcache.buckets[bid].head; b = b->next) {
    if(b->refcnt == 0 && (victim == 0 || b->timestamp < victim->timestamp)) {
      victim = b;
    }
  }
  
  if(victim) {
    // 在当前桶中找到可重用的缓冲区
    goto found;
  }

  // 当前桶没有可用缓冲区，需要从其他桶偷取
  release(&bcache.buckets[bid].lock);

  // 遍历所有桶寻找LRU缓冲区
  int victim_bucket = -1;
  victim = 0;
  
  // 按顺序获取锁，避免死锁
  for(int i = 0; i < NBUCKET; i++) {
    acquire(&bcache.buckets[i].lock);
    
    // 在当前桶中寻找LRU缓冲区
    for(b = bcache.buckets[i].head.next; b != &bcache.buckets[i].head; b = b->next) {
      if(b->refcnt == 0 && (victim == 0 || b->timestamp < victim->timestamp)) {
        victim = b;
        victim_bucket = i;
      }
    }
    
    // 如果我们找到了缓冲区，就停止搜索
    if(victim) {
      // 保持当前桶的锁，稍后释放
      break;
    } else {
      // 释放这个桶的锁，继续搜索
      release(&bcache.buckets[i].lock);
    }
  }

  if(!victim) {
    panic("bget: no buffers");
  }

  // 从原桶中移除victim
  victim->next->prev = victim->prev;
  victim->prev->next = victim->next;
  
  // 重新获取目标桶的锁
  acquire(&bcache.buckets[bid].lock);
  
  // 将victim添加到目标桶
  victim->next = bcache.buckets[bid].head.next;
  victim->prev = &bcache.buckets[bid].head;
  bcache.buckets[bid].head.next->prev = victim;
  bcache.buckets[bid].head.next = victim;
  
  // 释放原桶的锁
  release(&bcache.buckets[victim_bucket].lock);

found:
  // 设置缓冲区信息
  victim->dev = dev;
  victim->blockno = blockno;
  victim->valid = 0;
  victim->refcnt = 1;
  victim->timestamp = ticks;
  
  release(&bcache.buckets[bid].lock);
  acquiresleep(&victim->lock);
  return victim;
}

// Return a locked buf with the contents of the indicated block.
struct buf*
bread(uint dev, uint blockno)
{
  struct buf *b;

  b = bget(dev, blockno);
  if(!b->valid) {
    virtio_disk_rw(b, 0);
    b->valid = 1;
  }
  return b;
}

// Write b's contents to disk.  Must be locked.
void
bwrite(struct buf *b)
{
  if(!holdingsleep(&b->lock))
    panic("bwrite");
  virtio_disk_rw(b, 1);
}

// Release a locked buffer.
// Move to the head of the most-recently-used list.
void
brelse(struct buf *b)
{
  if(!holdingsleep(&b->lock))
    panic("brelse");

  releasesleep(&b->lock);

  int bid = HASH(b->blockno);
  acquire(&bcache.buckets[bid].lock);
  b->refcnt--;
  if(b->refcnt == 0) {
    // 没有更多引用时更新时间戳
    b->timestamp = ticks;
  }
  release(&bcache.buckets[bid].lock);
}

void
bpin(struct buf *b) {
  int bid = HASH(b->blockno);
  acquire(&bcache.buckets[bid].lock);
  b->refcnt++;
  release(&bcache.buckets[bid].lock);
}

void
bunpin(struct buf *b) {
  int bid = HASH(b->blockno);
  acquire(&bcache.buckets[bid].lock);
  b->refcnt--;
  release(&bcache.buckets[bid].lock);
}