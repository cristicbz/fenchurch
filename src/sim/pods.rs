use super::simulation::Bodies;
use math::Vec3f;
use idcontain::{IdMap, Id, IdSlab};

derive_flat! {
    #[element(Pod, &PodRef, &mut PodMut)]
    #[access(&PodsRef, &mut PodsMut)]
    pub struct Pods {
        #[element(body)]
        pub bodies: Bodies,

        #[element(colour)]
        pub colours: Vec<Vec3f>,
    }
}

pub struct PodId(Id<()>);

pub struct PodSystem {
    ids: IdSlab<()>,
    pods: IdMap<(), Pods>,
}

impl PodSystem {
    pub fn new() -> Self {
        PodSystem {
            ids: IdSlab::new(),
            pods: IdMap::new(),
        }
    }

    pub fn with_capacity(capacity: usize) -> Self {
        PodSystem {
            ids: IdSlab::with_capacity(capacity),
            pods: IdMap::with_capacity(capacity),
        }
    }

    pub fn len(&self) -> usize {
        self.ids.len()
    }

    pub fn add(&mut self, pod: Pod) -> PodId {
        let id = self.ids.insert(());
        self.pods.insert(id, pod);
        PodId(id)
    }

    pub fn remove(&mut self, pod_id: PodId) -> Option<Pod> {
        let PodId(id) = pod_id;
        if self.ids.remove(id).is_some() {
            self.pods.remove(id)
        } else {
            None
        }
    }

    pub fn get(&self, pod_id: PodId) -> Option<PodRef> {
        self.pods.get(pod_id.0)
    }

    pub fn get_mut(&mut self, pod_id: PodId) -> Option<PodMut> {
        self.pods.get_mut(pod_id.0)
    }

    pub fn pods(&self) -> PodsRef {
        self.pods.access()
    }

    pub fn pods_mut(&mut self) -> PodsMut {
        self.pods.access_mut()
    }
}
