use math::{vec3, Vec3f, Mat4};

#[derive(Clone)]
pub struct Light {
    pub direction: Vec3f,
    pub colour: Vec3f,
}

impl Light {
    pub fn transformed(&self, modelview: &Mat4) -> Light {
        Light {
            direction: modelview.transform_direction(&self.direction).xyz(),
            colour: self.colour,
        }
    }
}


#[derive(Clone)]
pub struct Lights {
    pub key: Light,
    pub fill: Light,
    pub back: Light,
    pub ambient: Vec3f,
}

impl Default for Lights {
    fn default() -> Self {
        Lights {
            key: Light {
                direction: vec3(0.0, 1.0, 1.0),
                colour: vec3(1.0, 1.0, 1.0) * 0.5,
            },
            fill: Light {
                direction: vec3(1.0, 1.0, 0.0),
                colour: vec3(1.0, 0.8, 0.7) * 0.4,
            },
            back: Light {
                direction: vec3(-1.0, -1.0, 0.0),
                colour: vec3(0.7, 0.8, 1.0) * 0.1,
            },
            ambient: vec3(0.2, 1.0, 1.0) * 0.05,
        }
    }
}

impl Lights {
    pub fn transformed(&self, modelview: &Mat4) -> Lights {
        Lights {
            key: self.key.transformed(modelview),
            fill: self.fill.transformed(modelview),
            back: self.back.transformed(modelview),
            ambient: self.ambient,
        }
    }
}
