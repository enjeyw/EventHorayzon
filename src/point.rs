use std::ops::{ Add, Mul, Sub };

#[derive(Copy, Clone)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64
}

impl Point {

    pub fn elems(&self) ->  (f64, f64, f64) {
        (self.x, self.y, self.z)
    }

    pub fn mag(self) -> f64 {
        (self.x.powi(2) + self.y.powi(2) + self.z.powi(2)).sqrt()
    }

    pub fn unit(self) -> Point {
        self * (1.0 / self.mag())
    }

    pub fn dot(&self, p: Point) -> f64 {
        let (x0, y0, z0) = self.elems();
        let (x1, y1, z1) = p.elems();
        x0*x1 + y0*y1 + z0*z1
    }

    pub fn cross(&self, p: Point) -> Point {
        let (a1, a2, a3) = self.elems();
        let (b1, b2, b3) = p.elems();

        Point {
            x: a2 * b3 - a3 * b2,
            y: -(a1 * b3 - a3 * b1),
            z: a1 * b2 - a2 * b1
        }

    }
}

impl Add for Point {
    type Output = Point;

    fn add(self, _rhs: Point) -> Point {
        Point {
            x: self.x + _rhs.x,
            y: self.y + _rhs.y,
            z: self.z + _rhs.z
        }
    }
}

impl Sub for Point {
    type Output = Point;

    fn sub(self, _rhs: Point) -> Point {
        Point {
            x: self.x - _rhs.x,
            y: self.y - _rhs.y,
            z: self.z - _rhs.z
        }
    }
}

impl Mul<f64> for Point {
    type Output = Point;

    fn mul(self, _rhs: f64) -> Point {
        let (x, y, z) = self.elems();
        Point {
            x: x * _rhs,
            y: y * _rhs,
            z: z * _rhs
        }
    }
}

impl Mul<Point> for f64 {
    type Output = Point;

    fn mul(self, _rhs: Point) -> Point {
        let (x, y, z) = _rhs.elems();
        Point {
            x: x * self,
            y: y * self,
            z: z * self
        }
    }
}