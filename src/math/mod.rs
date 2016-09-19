pub mod mat;
pub mod vector;
pub mod sphere;
pub mod contact;
mod aabb;

pub use self::mat::Mat4;
pub use self::vector::{Vector, Field, Vec2, Vec2f, Vec3, Vec3f, Vec4, Vec4f};
pub use self::aabb::Aabb;
pub use self::contact::ContactInfo;
pub use self::sphere::Sphere;

pub fn clamp<T: PartialOrd>(value: T, (limit_min, limit_max): (T, T)) -> T {
    if value < limit_min {
        limit_min
    } else if value > limit_max {
        limit_max
    } else {
        value
    }
}
