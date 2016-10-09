use std::u8;

pub const NUM_STATES: u8 = u8::MAX;
pub type StateOffset = i8;

pub enum Input {
    Timer,
}

pub enum Action {
    Noop,
    SetTimer { duration: f32 },
    ChangeSize { ratio: f32 },
}

pub struct Transition {
    pub action: Action,
    pub state_offset: StateOffset,
}

struct TransitionTableIndex {
    index: u32,
}

pub struct TransitionTableId(Id<TransitionTableIndex>);

struct MachineIndex {
    index: u32,
}

pub struct MachineId(Id<MachineIndex>);

pub struct Machines {
    tables: IdVec<TransitionTableIndex>,
    transitions: Vec<Transition>,

    states: IdVec<MachineIndex>,
    machines: Vec<
}
