use num::{Float, One, Zero};
use std::ops::{Add, Div, Index, IndexMut, Mul, Neg, Sub, AddAssign, MulAssign, DivAssign,
               SubAssign};

pub type Vec2f = Vec2<f32>;
pub type Vec3f = Vec3<f32>;
pub type Vec4f = Vec4<f32>;

pub trait Vector: Mul<<Self as Vector>::Scalar, Output=Self>
                + MulAssign<<Self as Vector>::Scalar>
                + MulAssign<Self>
                + Div<<Self as Vector>::Scalar, Output=Self>
                + DivAssign<<Self as Vector>::Scalar>
                + DivAssign<Self>
                + Add<Output=Self>
                + AddAssign
                + Sub<Output=Self> + Zero
                + SubAssign
                + Clone + PartialEq + PartialOrd
                + Index<usize, Output=<Self as Vector>::Scalar>
                + IndexMut<usize> {

    type Scalar: Field;

    fn dot(&self, rhs: &Self) -> Self::Scalar;

    #[inline]
    fn squared_norm(&self) -> Self::Scalar {
        self.dot(self)
    }

    #[inline]
    fn norm(&self) -> Self::Scalar
        where Self::Scalar: Float
    {
        self.squared_norm().sqrt()
    }

    #[inline]
    fn normalize(&mut self)
        where Self::Scalar: Float
    {
        let norm = self.norm();
        if norm == Self::Scalar::zero() {
            *self = Self::zero()
        } else {
            *self /= norm;
        }
    }

    #[inline]
    fn normalized(mut self) -> Self
        where Self::Scalar: Float
    {
        self.normalize();
        self
    }
}


pub trait Field: Mul<Output=Self> + Div<Output=Self>
               + Add<Output=Self> + Sub<Output=Self>
               + MulAssign + DivAssign + AddAssign + SubAssign
               + Zero + One + Copy + Clone + PartialEq + PartialOrd {}

impl<S> Field for S
    where S: Mul<Output=S> + Div<Output=S>
           + Add<Output=S> + Sub<Output=S>
           + MulAssign + DivAssign + AddAssign + SubAssign
           + Zero + One + Copy + PartialEq + PartialOrd {}


// Vec2

macro_rules! impl_vectors {
    ($(#[vector_indices($($index:expr),+)]
       pub struct $name:ident <Scalar: Field> ([Scalar; $size:expr]);)+) => {
        $(impl_vector!($name[$size]: $($index),+);)+
    }
}


macro_rules! impl_vector {
    ($name:ident[$size:expr]: $($index:expr),+) => {
        #[repr(C)]
        #[derive(Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Debug, Default)]
        pub struct $name<Scalar: Field>([Scalar; $size]);

        impl<Scalar: Field> Zero for $name<Scalar> {
            #[inline]
            fn zero() -> Self {
                $name([Zero::zero(); $size])
            }

            #[inline]
            fn is_zero(&self) -> bool {
                true $(&& self.0[$index].is_zero())+
            }
        }

        impl<Scalar: Field + Neg<Output = Scalar>> Neg for $name<Scalar> {
            type Output = Self;

            #[inline]
            fn neg(self) -> Self {
                $name([$(-self.0[$index],)+])
            }
        }

        impl<Scalar: Field> Vector for $name<Scalar> {
            type Scalar = Scalar;

            #[inline]
            fn dot(&self, rhs: &Self) -> Scalar {
                Scalar::zero() $(+ self.0[$index] * rhs.0[$index])+
            }
        }

        // By-value arithmetic.
        impl<Scalar: Field> Mul<Scalar> for $name<Scalar> {
            type Output = Self;

            #[inline]
            fn mul(self, rhs: Scalar) -> Self {
                $name([$(self.0[$index] * rhs),+])
            }
        }

        impl<Scalar: Field> Mul<$name<Scalar>> for $name<Scalar> {
            type Output = Self;

            #[inline]
            fn mul(self, rhs: Self) -> Self {
                $name([$(self.0[$index] * rhs.0[$index]),+])
            }
        }

        impl<Scalar: Field> MulAssign<Scalar> for $name<Scalar> {
            #[inline]
            fn mul_assign(&mut self, rhs: Scalar) {
                $(self.0[$index] *= rhs;)+
            }
        }

        impl<Scalar: Field> MulAssign<$name<Scalar>> for $name<Scalar> {
            #[inline]
            fn mul_assign(&mut self, rhs: Self) {
                $(self.0[$index] *= rhs.0[$index];)+
            }
        }

        impl<Scalar: Field> Div<Scalar> for $name<Scalar> {
            type Output = Self;

            #[inline]
            fn div(self, rhs: Scalar) -> Self {
                let inv_rhs = Scalar::one() / rhs;
                $name([$(self.0[$index] * inv_rhs),+])
            }
        }

        impl<Scalar: Field> Div<$name<Scalar>> for $name<Scalar> {
            type Output = Self;

            #[inline]
            fn div(self, rhs: Self) -> Self {
                $name([$(self.0[$index] / rhs.0[$index]),+])
            }
        }

        impl<Scalar: Field> DivAssign<Scalar> for $name<Scalar> {
            #[inline]
            fn div_assign(&mut self, rhs: Scalar) {
                let inv_rhs = Scalar::one() / rhs;
                $(self.0[$index] *= inv_rhs;)+
            }
        }

        impl<Scalar: Field> DivAssign<$name<Scalar>> for $name<Scalar> {
            #[inline]
            fn div_assign(&mut self, rhs: Self) {
                $(self.0[$index] /= rhs.0[$index];)+
            }
        }

        impl<Scalar: Field> Add<Scalar> for $name<Scalar> {
            type Output = Self;

            #[inline]
            fn add(self, rhs: Scalar) -> Self {
                $name([$(self.0[$index] + rhs),+])
            }
        }

        impl<Scalar: Field> Add<$name<Scalar>> for $name<Scalar> {
            type Output = Self;

            #[inline]
            fn add(self, rhs: $name<Scalar>) -> Self {
                $name([$(self.0[$index] + rhs.0[$index]),+])
            }
        }

        impl<Scalar: Field> AddAssign<Scalar> for $name<Scalar> {
            #[inline]
            fn add_assign(&mut self, rhs: Scalar) {
                $(self.0[$index] += rhs;)+
            }
        }

        impl<Scalar: Field> AddAssign<$name<Scalar>> for $name<Scalar> {
            #[inline]
            fn add_assign(&mut self, rhs: $name<Scalar>) {
                $(self.0[$index] += rhs.0[$index];)+
            }
        }

        impl<Scalar: Field> Sub<$name<Scalar>> for $name<Scalar> {
            type Output = Self;

            #[inline]
            fn sub(self, rhs: $name<Scalar>) -> Self {
                $name([$(self.0[$index] - rhs.0[$index]),+])
            }
        }

        impl<Scalar: Field> Sub<Scalar> for $name<Scalar> {
            type Output = Self;

            #[inline]
            fn sub(self, rhs: Scalar) -> Self {
                $name([$(self.0[$index] - rhs),+])
            }
        }

        impl<Scalar: Field> SubAssign<$name<Scalar>> for $name<Scalar> {
            #[inline]
            fn sub_assign(&mut self, rhs: $name<Scalar>) {
                $(self.0[$index] -= rhs.0[$index];)+
            }
        }

        impl<Scalar: Field> SubAssign<Scalar> for $name<Scalar> {
            #[inline]
            fn sub_assign(&mut self, rhs: Scalar) {
                $(self.0[$index] -= rhs;)+
            }
        }

        // Other ops.
        impl<Scalar: Field> Index<usize> for $name<Scalar> {
            type Output = Scalar;

            #[inline]
            fn index(&self, index: usize) -> &Scalar {
                &self.0[index]
            }
        }

        impl<Scalar: Field> IndexMut<usize> for $name<Scalar> {
            #[inline]
            fn index_mut(&mut self, index: usize) -> &mut Scalar {
                &mut self.0[index]
            }
        }
    }
}

impl_vectors! {
    #[vector_indices(0, 1)]
    pub struct Vec2<Scalar: Field>([Scalar; 2]);

    #[vector_indices(0, 1, 2)]
    pub struct Vec3<Scalar: Field>([Scalar; 3]);

    #[vector_indices(0, 1, 2, 3)]
    pub struct Vec4<Scalar: Field>([Scalar; 4]);
}

impl<Scalar: Field> Vec2<Scalar> {
    #[inline]
    pub fn new(x: Scalar, y: Scalar) -> Self {
        Vec2([x, y])
    }

    #[inline]
    pub fn cross(&self, rhs: &Self) -> Scalar {
        self[0] * rhs[1] - self[1] * rhs[0]
    }

    #[inline]
    pub fn angle(&self) -> Scalar
        where Scalar: Float
    {
        self[1].atan2(self[0])
    }

    #[inline]
    pub fn normal(&self) -> Vec2<Scalar>
        where Scalar: Neg<Output = Scalar>
    {
        Vec2::new(-self[1], self[0])
    }

    #[inline]
    pub fn swap(&mut self) {
        self.0.swap(0, 1)
    }
}

impl<Scalar: Field> Vec3<Scalar> {
    #[inline]
    pub fn new(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Vec3([x, y, z])
    }

    #[inline]
    pub fn cross(&self, rhs: &Vec3<Scalar>) -> Self {
        let (lx, ly, lz) = (self[0], self[1], self[2]);
        let (rx, ry, rz) = (rhs[0], rhs[1], rhs[2]);
        Vec3::new(ly * rz - lz * ry, lz * rx - lx * rz, lx * ry - ly * rx)
    }

    #[inline]
    pub fn array(&self) -> &[Scalar; 3] {
        &self.0
    }
}

impl<Scalar: Field> Vec4<Scalar> {
    #[inline]
    pub fn new(x: Scalar, y: Scalar, z: Scalar, w: Scalar) -> Self {
        Vec4([x, y, z, w])
    }

    #[inline]
    pub fn xyz(&self) -> Vec3<Scalar> {
        Vec3([self.0[0], self.0[1], self.0[2]])
    }

    #[inline]
    pub fn array(&self) -> &[Scalar; 4] {
        &self.0
    }
}
