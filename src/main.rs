extern crate image;

use std::ops::{ Add, Mul, Sub };
use std::cmp::{ min };
use image::{ ImageBuffer };

use ndarray::prelude::*;


#[derive(Copy, Clone)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64
}

impl Point {

    fn elems(&self) ->  (f64, f64, f64) {
        (self.x, self.y, self.z)
    }

    fn mag(self) -> f64 {
        (self.x.powi(2) + self.y.powi(2) + self.z.powi(2)).sqrt()
    }

    fn unit(self) -> Point {
        self * (1.0 / self.mag())
    }

    fn dot(&self, p: Point) -> f64 {
        let (x0, y0, z0) = self.elems();
        let (x1, y1, z1) = p.elems();
        x0*x1 + y0*y1 + z0*z1
    }

    fn cross(&self, p: Point) -> Point {
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

pub struct Ray {
    // P = P0 + Vt
    pub direction: Point,
    pub origin: Point,
    pub magnitude: f64
}

impl Ray {
    fn new(direction: Point, origin: Point) -> Ray {
        Ray{
            direction: direction.unit(),
            origin: origin,
            magnitude: direction.mag()
        }
    }
}

pub struct LightSource {
    pub origin: Point,
    pub color: [f64; 3]
}

#[derive(Copy, Clone)]
pub struct Intercept {
    pub location: Point,
    pub distance: f64,
    // pub norm: Point
}

pub trait Surface {
    fn intercept(&self, ray: &Ray) -> Option<Intercept>;
    fn get_norm(&self, location: Point) -> Point;
    fn get_relfectivity(&self) -> f64;
    fn get_transmission_props(&self) -> (f64, f64);
}

pub struct Sphere {
    // (P - P0)^2 -  r^2 = 0
    pub center: Point,
    pub radius: f64,
    pub reflectivity: f64, 
    pub transmissivity: f64,
    pub refractive_index: f64,
}

impl Surface for Sphere {
    fn intercept(&self, ray: &Ray) -> Option<Intercept> {
        let oc = ray.origin - self.center;
        let a = ray.direction.dot(ray.direction);
        let b = 2.0 * oc.dot(ray.direction);
        let c = oc.dot(oc) - self.radius.powi(2);
        let discriminant = b*b - 4.0*a*c;
        if discriminant < 0.0 {
            None
        } else {
            let t = (-b - discriminant.sqrt()) / (2.0*a);

            if t < 0.0 {
                return None
            }

            let p = ray.origin + t * ray.direction;
            Some(Intercept{location: p, distance: t})
        }
    }

    fn get_norm(&self, location: Point) -> Point {
        (location - self.center).unit()
    }

    fn get_relfectivity(&self) -> f64 {
        self.reflectivity
    }

    fn get_transmission_props(&self) -> (f64, f64) {
        (self.transmissivity, self.refractive_index)
    }
}

pub struct Plane {
    // P * norm + d = 0`
    pub norm: Point,
    pub d: f64,
    pub bounds: Option<[Point; 3]>,
    pub reflectivity: f64,
    pub transmissivity: f64,
    pub refractive_index: f64
}

impl Plane {
    fn point_is_inside(&self, bounds: [Point; 3], ray: &Ray, intercept: Point) -> bool {
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

    fn add_light_effects(&self, reflectivity: f64, transmissivity: f64, refractive_index: f64) -> Plane {
        Plane {
            norm: self.norm,
            d: self.d,
            bounds: self.bounds,
            reflectivity: reflectivity,
            transmissivity: transmissivity,
            refractive_index: refractive_index
        }
    }

    fn new(bounds: [Point; 3]) -> Plane {
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
            refractive_index: 0.0
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
}

pub struct ViewPoint {
    pub origin: Point,
    pub direction: Point,
    pub distance: f64,
    pub width: f64,
    pub resolution: (u32, u32),
}

pub struct ViewPointDeets {
    pub top_left: Point,
    pub step_size: f64,
    pub lengthways: Point,
    pub upwards: Point
}

impl ViewPoint {
    fn get_viewpoint_deets(&self) -> ViewPointDeets {
        let step_size = self.width / (self.resolution.0 as f64);

        let height = step_size * (self.resolution.1 as f64);

        let unit_dir = self.direction.unit();

        let centroid = self.origin + unit_dir * self.distance; 

        // Find lengthways vector by cross-producting the direction with a vertical vector
        // Right hand rule means that length should be in positive direction in this ordering
        let lengthways = unit_dir.cross(Point{ x: 0.0, y: 0.0, z: 1.0 }).unit();

        // Similar deal for upwards dir
        // Right hand rule means that upwards should be in positive direction in this ordering
        let upwards = lengthways.cross(unit_dir).unit();

        let top_left = centroid + upwards * (height / 2.0) + lengthways * ( -1.0 * self.width / 2.0);

        ViewPointDeets{
            top_left: top_left,
            step_size: step_size,
            lengthways: lengthways,
            upwards: upwards
        }
    
    }
}

type SurfaceBox = Box<dyn Surface>;

fn project_ray(ray: Ray, surfaces: &Vec<SurfaceBox>, light_sources: &Vec<LightSource>, refractive_index: f64, bounces_remaining: u8, origin_surface: Option<&SurfaceBox>) -> (f64, f64, f64) {
    let mut combined_values = (0.0, 0.0, 0.0);
    let mut light_values = (0.0, 0.0, 0.0);
    let mut bounce_values = (0.0, 0.0, 0.0);
    let mut transmit_values = (0.0, 0.0, 0.0);


    let mut smallest_intercept = None;
    for plane in surfaces {

        let check_for_collision = match origin_surface {
            None => true,
            Some(os) => !std::ptr::eq(os, plane)
        };

        // if ! check_for_collision {
        //     dbg!("Not checking");
        // }

        if check_for_collision {
            match plane.intercept(&ray) {
                None => {},
                Some(intercept) => {
                    match smallest_intercept {
                        None => { 
                            smallest_intercept = Some((intercept, plane));
                        
                        },
                        Some(existing) => {
                            if intercept.distance < existing.0.distance {
                                smallest_intercept = Some((intercept, plane))
                            }
                        }
                    }
                }   
            };
        }
    }

    match smallest_intercept {
        None => {},
        Some((intercept, surf)) => {

            let reflectivity = surf.get_relfectivity();

            let norm = surf.get_norm(intercept.location);
            let inbound = ray.direction;


            if reflectivity > 0.0 && bounces_remaining > 0 {

                // dbg!(inbound.elems());
                // dbg!(norm.elems());
                // dbg!(t_ray.direction.elems());

                let bounce_dir = inbound - 2.0 * (inbound.dot(norm) * norm);
                let b_ray = Ray::new(bounce_dir, intercept.location);

                bounce_values = project_ray(b_ray, surfaces, light_sources, refractive_index, bounces_remaining - 1, Some(&Box::new(surf)));
            }

            let (transmissivity, n2_refractive_index) = surf.get_transmission_props();

            if transmissivity > 0.0 {
                let mut mu = refractive_index/n2_refractive_index;
                if n2_refractive_index == refractive_index {
                    // dbg!("flipping back");
                    mu = n2_refractive_index/1.0;
                }

                let t_dir = (1.0 - mu*mu * (1.0 - norm.dot(inbound).powi(2))).sqrt() * norm + mu * ( inbound - norm.dot(inbound) * norm);

                let t_ray = Ray::new(t_dir, intercept.location);

                // dbg!(inbound.elems());
                // dbg!(norm.elems());
                // dbg!(t_ray.direction.elems());

                transmit_values = project_ray(t_ray, surfaces, light_sources, n2_refractive_index, bounces_remaining, None);
                
            };

            for source in light_sources {
                let source_dir = source.origin - intercept.location;
                let ray_to_source = Ray::new(source_dir, intercept.location);
 
                let mut has_any_collision = false;
                for obstruction_plane in surfaces {
                    if !std::ptr::eq(surf, obstruction_plane) {
                        match obstruction_plane.intercept(&ray_to_source) {
                            None => {},
                            Some(_intercept) => {
                                has_any_collision = true;
                            }
                        }
                    }
                }

                if !has_any_collision {

                    let intensity = ray_to_source.direction.dot(surf.get_norm(intercept.location)).abs() * 2.0 / ray_to_source.magnitude.cbrt();

                    light_values = (
                        source.color[0] * intensity + light_values.0,
                        source.color[1] * intensity + light_values.1,
                        source.color[2] * intensity + light_values.2                                            
                    )
                }
            }

            combined_values = (
                reflectivity * bounce_values.0 + (1.0 - reflectivity) * transmissivity * transmit_values.0 + (1.0 - reflectivity - (1.0 - reflectivity) * transmissivity) * light_values.0,
                reflectivity * bounce_values.1 + (1.0 - reflectivity) * transmissivity * transmit_values.1 + (1.0 - reflectivity - (1.0 - reflectivity) * transmissivity) * light_values.1,
                reflectivity * bounce_values.2 + (1.0 - reflectivity) * transmissivity * transmit_values.2 + (1.0 - reflectivity - (1.0 - reflectivity) * transmissivity) * light_values.2,
            );

        }
    }
    
    combined_values
}

// fn combine_values(reflectivity: f64, transmissivity: f64, index: usize, bounce_values: (f64, f64, f64), light_values: (f64, f64, f64), transmit_values: (f64, f64, f64)) -> f64 {
//     reflectivity * bounce_values[index] + (1.0 - reflectivity) * light_values.index
// }

fn render(surfaces: Vec<SurfaceBox>, light_sources: Vec<LightSource>, viewpoint: ViewPoint) {
    let mut img = ImageBuffer::from_fn(viewpoint.resolution.0, viewpoint.resolution.1, |_x, _y| { 
        image::Rgb([0u8,0u8,0u8])
    });

    let mut imbuff = Array::from_elem((viewpoint.resolution.0 as usize, viewpoint.resolution.1 as usize), (0.0, 0.0, 0.0));

    let deets = viewpoint.get_viewpoint_deets();

    let start_point = deets.top_left;

    for j in 0..viewpoint.resolution.1 {
        for i in 0..viewpoint.resolution.0 {
            let current_point =  start_point + (i as f64) * deets.step_size * deets.lengthways + (j as f64) * deets.step_size * (-1.0 * deets.upwards);
            let ray_dir = current_point - viewpoint.origin;
            let ray = Ray::new(ray_dir, viewpoint.origin);
            
            let bounces_remaining = 2;
            imbuff[[i as usize,j as usize]] = project_ray(ray, &surfaces, &light_sources, 1.0, bounces_remaining, None)
        }
    }

    for j in 0..viewpoint.resolution.1 {
        for i in 0..viewpoint.resolution.0 {
            let existing_pix = imbuff[[i as usize,j as usize]];
                                    
            let pixel = image::Rgb([
                min(existing_pix.0 as u8, 255),
                min(existing_pix.1 as u8, 255),
                min(existing_pix.2 as u8, 255),
                ]);

            img.put_pixel(i, j, pixel);
        }
    }

    img.save("out/test.png").unwrap();

}

fn main() {

    let surfaces:Vec<SurfaceBox> = vec![
        Box::new(Sphere{ center: Point{ x: 2.5, y: -5.0, z: 5.0}, radius: 2.5, reflectivity: 0.0, transmissivity: 0.0, refractive_index: 0.0}),
        Box::new(Sphere{ center: Point{ x: 2.5, y: 0.0, z: 5.0}, radius: 1.0, reflectivity: 1.0, transmissivity: 0.0, refractive_index: 0.0}),
        Box::new(Sphere{ center: Point{ x: 1.5, y: -12.0, z: 5.0}, radius: 1.0, reflectivity: 0.0, transmissivity: 0.6, refractive_index: 1.4}),

        // Box::new(Plane::new(
        //     [
        //         Point{ x: 5.0, y: -5.0, z: 0.0},
        //         Point{ x: 5.0, y: -10.0, z: 0.0},
        //         Point{ x: 5.0, y: -10.0, z: 4.0}
        //     ]
        // ).add_light_effects(0.0, 0.9, 1.8)),

        // Box::new(Plane::new(
        //     [
        //         Point{ x: 5.1, y: -5.0, z: 0.0},
        //         Point{ x: 5.1, y: -10.0, z: 0.0},
        //         Point{ x: 5.1, y: -10.0, z: 4.0}
        //     ]
        // ).add_light_effects(0.0, 0.9, 1.8)),

        Box::new(Plane::new(
            [
                Point{ x: 5.0, y: 0.0, z: 0.0},
                Point{ x: 0.0, y: 5.0, z: 0.0},
                Point{ x: 0.0, y: 0.0, z: 5.0}
            ]
        )),
        Box::new(Plane::new(
            [
                Point{ x: 5.0, y: 0.0, z: 0.0},
                Point{ x: 0.0, y: -5.0, z: 0.0},
                Point{ x: 0.0, y: 0.0, z: -5.0}
            ]
        )),
        Box::new(Plane::new(
            [
                Point{ x: 5.0, y: 0.0, z: 0.0},
                Point{ x: 0.0, y: -5.0, z: 0.0},
                Point{ x: 0.0, y: 0.0, z: 5.0}
            ]
        ).add_light_effects(0.0, 0.0, 0.0)),
        Box::new(Plane::new(
            [
                Point{ x: 5.0, y: 0.0, z: 0.0},
                Point{ x: 0.0, y: 5.0, z: 0.0},
                Point{ x: 0.0, y: 0.0, z: -5.0}
            ]
        )),
        Box::new(Plane::new(
            [
                Point{ x: -10.0, y: 500.0, z: 500.0},
                Point{ x: -10.0, y: -500.0, z: -500.0},
                Point{ x: -10.0, y: 500.0, z: -500.0},

            ]
        )),
        Box::new(Plane::new(
            [
                Point{ x: -10.0, y: 500.0, z: 500.0},
                Point{ x: -10.0, y: -500.0, z: -500.0},
                Point{ x: -10.0, y: -500.0, z: 500.0},
            ]
        ))
    ];

    let light_sources = vec![
        LightSource{
            origin: Point{ x: 20.0, y: 7.0, z: 9.0 }, color: [150.0, 0.0, 150.0]
        },
        LightSource{
            origin: Point{ x: 20.0, y: -3.0, z: 6.0 }, color: [0.0, 100.0, 0.0]
        },
        LightSource{
            origin: Point{ x: 5.0, y: -10.0, z: -10.0 }, color: [130.0, 130.0, 130.0]
        }
    ];

    let viewpoint = ViewPoint{ 
        direction: Point { x: -1.0, y: 0.1, z: 0.1},
        origin: Point{ x: 12.0, y: -3.0, z: -1.0},
        distance: 1.0,
        width: 3.0,
        resolution: (1000, 1000)
    };

    render(surfaces, light_sources, viewpoint);
    
}
