use crate::point::{Point};

#[derive(Copy, Clone)]
pub struct Ray {
    // P = P0 + Vt
    pub direction: Point,
    pub origin: Point,
    pub magnitude: f64
}

impl Ray {
    pub fn new(direction: Point, origin: Point) -> Ray {
        Ray{
            direction: direction.unit(),
            origin: origin,
            magnitude: direction.mag()
        }
    }
}

#[derive(Copy, Clone)]
pub struct Intercept {
    pub location: Point,
    pub distance: f64,
    // pub norm: Point
}

#[derive(Clone)]
pub struct Illumination {
    pub locations: Vec<Point>,
    pub color: [f64; 3]
}

pub trait Surface {
    fn intercept(&self, ray: &Ray) -> Option<Intercept>;
    fn get_norm(&self, location: Point) -> Point;
    fn get_relfectivity(&self) -> f64;
    fn get_transmission_props(&self) -> (f64, f64);
    fn centroidish(&self) -> Point;
    fn search_radius(&self) -> f64;
    fn get_has_light(&self) -> bool;
    fn set_has_light(&mut self, h: bool);
    fn get_illuminations(&self) -> &Vec<Illumination>;
    fn add_illumination(&mut self, illumination: Illumination);
}

#[derive(Clone)]
pub struct Sphere {
    // (P - P0)^2 -  r^2 = 0
    pub center: Point,
    pub radius: f64,
    pub reflectivity: f64, 
    pub transmissivity: f64,
    pub refractive_index: f64,
    pub has_light: bool,
    pub illuminations: Vec<Illumination>
}

impl Surface for Sphere {
    fn intercept(&self, ray: &Ray) -> Option<Intercept> {
        let mut ray_origin = ray.origin;
        let mut oc = ray_origin - self.center;
        // epsilon hack :/ 
        if oc.mag() - self.radius < 0.00001 {
            ray_origin = ray_origin + ray.direction * 0.00001;
            oc = ray_origin - self.center;
        }

        let a = ray.direction.dot(ray.direction);
        let b = 2.0 * oc.dot(ray.direction);
        let c = oc.dot(oc) - self.radius.powi(2);
        let discriminant = b*b - 4.0*a*c;
        if discriminant < 0.0 {
            None
        } else {

            let mut t = (-b - discriminant.sqrt()) / (2.0*a);

            if t < 0.0 {
                t = (-b + discriminant.sqrt()) / (2.0*a);

                if t < 0.0 {
                    return None
                }
            }

            let p = ray_origin + t * ray.direction;

            // dbg!(t);
            // dbg!(p.elems());

            Some(Intercept{location: p, distance: t})
        }
    }

    fn get_norm(&self, location: Point) -> Point {
        -1.0 * (location - self.center).unit()
    }

    fn get_relfectivity(&self) -> f64 {
        self.reflectivity
    }

    fn get_transmission_props(&self) -> (f64, f64) {
        (self.transmissivity, self.refractive_index)
    }

    fn centroidish(&self) -> Point {
        self.center
    }

    fn search_radius(&self) -> f64 {
        self.radius
    }

    fn get_has_light(&self) -> bool {
        self.has_light
    }

    fn set_has_light(&mut self, h: bool){
        self.has_light = h;
    }

    fn get_illuminations(&self) -> &Vec<Illumination> {
        &self.illuminations
    }

    fn add_illumination(&mut self, illumination: Illumination) {
        self.illuminations.push(illumination)
    }
}

pub struct Plane {
    // P * norm + d = 0`
    pub norm: Point,
    pub d: f64,
    pub bounds: Option<[Point; 3]>,
    pub reflectivity: f64,
    pub transmissivity: f64,
    pub refractive_index: f64,
    pub illuminations: Vec<Illumination>
}

impl Plane {
    pub fn point_is_inside(&self, bounds: [Point; 3], ray: &Ray, intercept: Point) -> bool {
        let p0 = ray.origin;

        let all = (
            Plane::_check_side(bounds[0], bounds[1], bounds[2], intercept, p0),
            Plane::_check_side(bounds[1], bounds[2], bounds[0], intercept, p0),
            Plane::_check_side(bounds[2], bounds[0], bounds[1], intercept, p0)
        );

        all.0 && all.1 && all.2

        // Plane::_check_side(bounds[0], bounds[1], intercept, p0) &&
        // Plane::_check_side(bounds[1], bounds[2], intercept, p0) &&
        // Plane::_check_side(bounds[2], bounds[0], intercept, p0)
       
    }

    fn _check_side(t1: Point, t2: Point, t_opp: Point, p: Point, p0: Point) -> bool {

        let v1 = t1 - p0;
        let v2 = t2 - p0;
        let v_opp = t_opp - p0;
        let mut n1 = v2.cross(v1);

        let opposite_dot = n1.dot(v_opp);

        if opposite_dot < 0.0 {
            n1 = n1 * -1.0;
        }

        let unit_dir = n1.unit();

        let d1 = -1.0 * p0.dot(unit_dir);
        (p.dot(unit_dir) + d1) > 0.0 
    }

    pub fn add_light_effects(&self, reflectivity: f64, transmissivity: f64, refractive_index: f64) -> Plane {
        Plane {
            norm: self.norm,
            d: self.d,
            bounds: self.bounds,
            reflectivity: reflectivity,
            transmissivity: transmissivity,
            refractive_index: refractive_index,
            illuminations: Vec::new()
        }
    }

    pub fn new(bounds: [Point; 3]) -> Plane {
        let v1 = bounds[1] - bounds[0];
        let v2 = bounds[2] - bounds[0];
        let n = v1.cross(v2).unit();

        let d = - bounds[0].dot(n);

        Plane {
            norm: n,
            d: d,
            bounds: Some(bounds),
            reflectivity: 0.0,
            transmissivity: 0.0,
            refractive_index: 0.0,
            illuminations: Vec::new()
        }
    }

}

impl Surface for Plane {
    fn intercept(&self, ray: &Ray) -> Option<Intercept> {

        let dir_dot_n = ray.direction.dot(self.norm);

        if dir_dot_n == 0.0 {
            return None;
        }

        let t = -(ray.origin.dot(self.norm) + self.d) / dir_dot_n;

        if t <= 0.0 {
           return None;
        }
        
        let p = ray.origin + t * ray.direction;

        match self.bounds {
            None => Some(Intercept{location: p, distance: t}),
            Some(bounds) => {
                if self.point_is_inside(bounds, ray, p) {
                    Some(Intercept{location: p, distance: t})
                } else {
                    None
                }
            }
        }
    }

    fn get_norm(&self, _location: Point) -> Point {
        self.norm
    }

    fn get_relfectivity(&self) -> f64 {
        self.reflectivity
    }

    fn get_transmission_props(&self) -> (f64, f64) {
        (self.transmissivity, self.refractive_index)
    }

    fn centroidish(&self) -> Point {
        // Really bad version for now just to see if the concept even works
        match self.bounds {
            Some(bounds) => bounds[0],
            None => Point{x: 0.0, y: 0.0, z: 0.0}
        }
    }

    fn search_radius(&self) -> f64 {
        match self.bounds {
            Some(bounds) => { 
                let b01 = (bounds[0] - bounds[1]).mag();
                let b20 = (bounds[2] - bounds[0]).mag();
                if b01 > b20 {
                    b01
                } else {
                    b20
                }
            },
            None => 1.0
        }
    }

    fn get_has_light(&self) -> bool {
        false
    }

    fn set_has_light(&mut self, _h: bool){
    }

    fn get_illuminations(&self) -> &Vec<Illumination> {
        &self.illuminations
    }

    fn add_illumination(&mut self, illumination: Illumination) {
        self.illuminations.push(illumination)
    }
}

