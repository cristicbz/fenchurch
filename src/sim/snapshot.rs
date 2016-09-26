use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicUsize, Ordering};

pub struct Snapshot<T> {
    state: AtomicUsize,
    data: UnsafeCell<T>,
}

unsafe impl<T: Sync> Sync for Snapshot<T> {}
unsafe impl<T: Send> Send for Snapshot<T> {}

impl<T> Snapshot<T> {
    pub fn new(data: T) -> Self {
        Snapshot {
            state: AtomicUsize::new(SnapshotState::NeedsWrite as usize),
            data: UnsafeCell::new(data),
        }
    }

    pub fn read<F, U>(&self, with: F) -> Option<U>
        where F: FnOnce(&T) -> U
    {
        if self.state.compare_and_swap(SnapshotState::NeedsRead as usize,
                                       SnapshotState::Busy as usize,
                                       Ordering::AcqRel) ==
           SnapshotState::NeedsRead as usize {
            let result = with(unsafe { &*self.data.get() });
            self.state.store(SnapshotState::NeedsWrite as usize, Ordering::Release);
            Some(result)
        } else {
            None
        }
    }

    pub fn write<F, U>(&self, with: F) -> Option<U>
        where F: FnOnce(&mut T) -> U
    {
        if self.state.compare_and_swap(SnapshotState::NeedsWrite as usize,
                                       SnapshotState::Busy as usize,
                                       Ordering::AcqRel) ==
           SnapshotState::NeedsWrite as usize {
            let result = with(unsafe { &mut *self.data.get() });
            self.state.store(SnapshotState::NeedsRead as usize, Ordering::Release);
            Some(result)
        } else {
            None
        }
    }
}

#[repr(usize)]
enum SnapshotState {
    NeedsWrite,
    NeedsRead,
    Busy,
}
