use std::marker::PhantomData;
use std::sync::atomic::{AtomicUsize, Ordering};

pub struct AtomicMutIndexer<'a, T: 'a> {
    data: *mut T,
    len: usize,
    next: AtomicUsize,
    dummy: PhantomData<&'a mut [T]>,
}

unsafe impl<'a, T> Sync for AtomicMutIndexer<'a, T> {}

impl<'a, T: 'a> AtomicMutIndexer<'a, T> {
    pub fn new(slice: &'a mut [T]) -> Self {
        AtomicMutIndexer {
            data: slice.as_mut_ptr(),
            len: slice.len(),
            next: AtomicUsize::new(0),
            dummy: PhantomData,
        }
    }

    pub fn done(self) -> usize {
        self.next.load(Ordering::SeqCst)
    }

    pub fn get(&self) -> Option<(usize, &'a mut T)> {
        let index = self.next.fetch_add(1, Ordering::SeqCst);
        if index < self.len {
            Some((index, unsafe { &mut *self.data.offset(index as isize) }))
        } else {
            self.next.store(self.len, Ordering::SeqCst);
            None
        }
    }

    pub fn get2(&self) -> Option<(usize, &'a mut T, &'a mut T)> {
        let index = self.next.fetch_add(2, Ordering::SeqCst);
        if index < self.len - 1 {
            Some(unsafe {
                (index,
                 &mut *self.data.offset(index as isize),
                 &mut *self.data.offset((index + 1) as isize))
            })
        } else {
            self.next.store(self.len, Ordering::SeqCst);
            None
        }
    }
}
