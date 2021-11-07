use std::cmp::{ min };
use ndarray::prelude::*;
use image::{ ImageBuffer };

use crate::geometry::{Intercept, Ray, Surface};
use crate::point::{Point};

pub type SurfaceBox = Box<dyn Surface>;
type F64Color = [f64; 3];

pub struct LightSource {
    pub origin: Point,
    pub color: F64Color
}

pub struct ViewPoint {
    pub origin: Point,
    pub direction: Point,
    pub up: Point,
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
        let lengthways = unit_dir.cross(self.up).unit();

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

fn no_field(p: Point) -> Point {
    return Point{x: 0.0, y: 0.0, z: 0.0}
}

fn attractor(p: Point) -> Point {
    let hole_loc =  Point{x: 0.0, y: 0.0, z: 0.0};
    let hole_mass = 0.5;
    let dist = (p - hole_loc).mag();
    let scale = hole_mass/dist.powi(2);
    // let scale = 0.01;
    return Point{x: -p.x*scale, y: -p.y*scale, z: -p.z*scale}
}

fn raymarch_intercept<'a>(initial_ray: Ray, surfaces: &'a Vec<SurfaceBox>, vec_field: fn(Point) -> Point) -> Option<(Intercept, &'a SurfaceBox)> {

    let TIME_STEP = 1.00;
    let MAX_STEPS = 1000;
    let SMALLEST_DISTANCE = 0.00001;
    let mut ray = initial_ray; 

    let mut current_velocity = ray.direction * 1.0;
    let mut current_location = ray.origin;
    let mut speed = 1.0;

    for stepnumber in 0..MAX_STEPS {

        let distance = speed * TIME_STEP;

        let mut smallest_intercept = None;
        for plane in surfaces {
            match plane.intercept(&ray) {
                None => {},
                Some(intercept) => {
                    if intercept.distance > SMALLEST_DISTANCE && intercept.distance < distance {
                        match smallest_intercept {
                            None => { 
                                smallest_intercept = Some((intercept, plane));
                            },
                            Some(existing) => {
                                if intercept.distance < existing.0.distance {
                                    smallest_intercept = Some((intercept, plane));
                                }
                            }
                        }                   
                    }
                }   
            };
        }

        match smallest_intercept {
            Some((int, surf)) => {
                return Some((int, surf))
            }
            None => {}
        }

        

        // // Runge-Kutta
        // let c0 = current_velocity;
        // let k0 = vec_field(current_location);

        // let c1 = current_velocity + TIME_STEP/2.0 * k0;
        // let k1 = vec_field(current_location + TIME_STEP/2.0*c0);

        // let c2 = current_velocity + TIME_STEP/2.0 * k1;
        // let k2 = vec_field(current_location + TIME_STEP/2.0*c1);

        // let c3 = current_velocity + TIME_STEP/2.0 * k2;
        // let k3 = vec_field(current_location + TIME_STEP*c2);

        // let delta_location = TIME_STEP/6.0 * (c0 + 2.0*c1 + 2.0*c2 + c3);
        // let delta_velocity = TIME_STEP/6.0 * (k0 + 2.0*k1 + 2.0*k2 + k3);
        // current_location = current_location + delta_location;
        // current_velocity = current_velocity + delta_velocity;

        // Euler
        current_velocity = ray.direction * speed;
        let delta_location = current_velocity * TIME_STEP;
        current_location = current_location + delta_location;
        let acceleration = vec_field(current_location);
        current_velocity = current_velocity + acceleration * TIME_STEP; 

        // speed = current_velocity.mag();

        speed = 1.0;
        ray = Ray::new(current_velocity, current_location);


    }

    None
}    

pub fn project_ray(ray: Ray, surfaces: &Vec<SurfaceBox>, light_sources: &Vec<LightSource>, refractive_index: f64, bounces_remaining: u8) -> F64Color {
    let mut combined_values: F64Color = [0.0, 0.0, 0.0];
    let mut light_values: F64Color = [0.0, 0.0, 0.0];
    let mut bounce_values: F64Color = [0.0, 0.0, 0.0];
    let mut transmit_values: F64Color = [0.0, 0.0, 0.0];


    // let mut smallest_intercept = None;
    // for plane in surfaces {
    //     match plane.intercept(&ray) {
    //         None => {},
    //         Some(intercept) => {
    //             if intercept.distance > 0.001 {
    //                 match smallest_intercept {
    //                     None => { 
    //                         smallest_intercept = Some((intercept, plane)); 
    //                     },
    //                     Some(existing) => {
    //                         if intercept.distance < existing.0.distance {
    //                             smallest_intercept = Some((intercept, plane))
    //                         }
    //                     }
    //                 }
    //             }
    //         }   
    //     };
    // }

    let smallest_intercept = raymarch_intercept(ray.clone(), surfaces, attractor);

    match smallest_intercept {
        None => {},
        Some((intercept, surf)) => {

            let reflectivity = surf.get_relfectivity();
            let (transmissivity, n2_refractive_index) = surf.get_transmission_props();

            if bounces_remaining > 0 {
                let norm = surf.get_norm(intercept.location);
                let inbound = ray.direction;

                if reflectivity > 0.0 {

                    let bounce_dir = inbound - 2.0 * (inbound.dot(norm) * norm);
                    let b_ray = Ray::new(bounce_dir, intercept.location);
    
                    bounce_values = project_ray(b_ray, surfaces, light_sources, refractive_index, bounces_remaining - 1);
                }
        
                if transmissivity > 0.0 {
                    let mut mu = refractive_index/n2_refractive_index;
                    if n2_refractive_index == refractive_index {
                        mu = n2_refractive_index/1.0;
                    }
    
                    let mut tnorm = norm;
                    if norm.dot(inbound) < 0.0 {
                        tnorm = -1.0 * norm
                    }
    
                    let t_dir = (1.0 - mu*mu * (1.0 - tnorm.dot(inbound).powi(2))).sqrt() * tnorm + mu * ( inbound - tnorm.dot(inbound) * tnorm);
    
                    let t_ray = Ray::new(t_dir, intercept.location);
    
                    transmit_values = project_ray(t_ray, surfaces, light_sources, n2_refractive_index, bounces_remaining - 1);
                };
            }
            

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

                    light_values = [
                        source.color[0] * intensity + light_values[0],
                        source.color[1] * intensity + light_values[1],
                        source.color[2] * intensity + light_values[2]                                            
                    ]
                }
            }

            combined_values = [
                combine_values(0, reflectivity, transmissivity, bounce_values, light_values, transmit_values),
                combine_values(1, reflectivity, transmissivity, bounce_values, light_values, transmit_values),
                combine_values(2, reflectivity, transmissivity, bounce_values, light_values, transmit_values)
            ]
        }
    }
    
    combined_values
}

pub fn combine_values(index: usize, reflectivity: f64, transmissivity: f64, bounce_values: F64Color, light_values: F64Color, transmit_values: F64Color) -> f64 {
    reflectivity * bounce_values[index] + transmissivity * transmit_values[index] + (1.0 - reflectivity - (1.0 - reflectivity) * transmissivity) * light_values[index]
}

pub fn has_light_effect(surface: &SurfaceBox) -> bool {
    let reflectivity = surface.get_relfectivity();
    let (transmissivity, _) = surface.get_transmission_props();
    reflectivity > 0.0 || transmissivity > 0.0
}

pub fn illuminate_surfaces(surfaces: &mut Vec<SurfaceBox>, light_sources: &Vec<LightSource>) {

    for s in surfaces {
        if has_light_effect(s) {
            let target = s.centroidish();
            for l in light_sources {
                s.search_radius();
            }
        }
    }
}


pub fn render(surfaces: &mut Vec<SurfaceBox>, light_sources: Vec<LightSource>, viewpoint: ViewPoint) {

    // illuminate_surfaces(surfaces, &light_sources);
    
    let mut img = ImageBuffer   ::from_fn(viewpoint.resolution.0, viewpoint.resolution.1, |_x, _y| { 
        image::Rgb([0u8,0u8,0u8])
    });

    let mut imbuff = Array::from_elem((viewpoint.resolution.0 as usize, viewpoint.resolution.1 as usize), [0.0, 0.0, 0.0]);

    let deets = viewpoint.get_viewpoint_deets();

    let start_point = deets.top_left;

    for j in 0..viewpoint.resolution.1 {
        for i in 0..viewpoint.resolution.0 {
            let current_point =  start_point + (i as f64) * deets.step_size * deets.lengthways + (j as f64) * deets.step_size * (-1.0 * deets.upwards);
            let ray_dir = current_point - viewpoint.origin;
            let ray = Ray::new(ray_dir, viewpoint.origin);
            
            let bounces_remaining = 4;
            imbuff[[i as usize,j as usize]] = project_ray(ray, &surfaces, &light_sources, 1.0, bounces_remaining)
        }
    }

    for j in 0..viewpoint.resolution.1 {
        for i in 0..viewpoint.resolution.0 {
            let existing_pix = imbuff[[i as usize,j as usize]];
                                    
            let pixel = image::Rgb([
                min(existing_pix[0] as u8, 255),
                min(existing_pix[1] as u8, 255),
                min(existing_pix[2] as u8, 255),
                ]);

            img.put_pixel(i, j, pixel);
        }
    }

    img.save("out/test.png").unwrap();

}