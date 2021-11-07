extern crate image;

mod point;
use point::{Point};

mod geometry;
use geometry::{ Plane, Sphere};

mod render;
use render::{SurfaceBox, LightSource, ViewPoint, render};

fn main() {

    let mut surfaces:Vec<SurfaceBox> = vec![
        Box::new(Sphere{ center: Point{ x: 2.5, y: -5.0, z: 5.0}, radius: 2.5, reflectivity: 1.0, transmissivity: 0.0, refractive_index: 0.0, has_light: false, illuminations: Vec::new()}),
        Box::new(Sphere{ center: Point{ x: 2.5, y: 0.0, z: 5.0}, radius: 1.0, reflectivity: 0.0, transmissivity: 0.0, refractive_index: 0.0, has_light: false, illuminations: Vec::new()}),
        Box::new(Sphere{ center: Point{ x: 2.5, y: -6.0, z: -2.0}, radius: 2.0, reflectivity: 0.4, transmissivity: 1.0, refractive_index: 1.5, has_light: false, illuminations: Vec::new()}),
        Box::new(Plane::new(
            [
                Point{ x: 5.0, y: -5.0, z: 0.0},  //Right
                Point{ x: 5.0, y: -10.0, z: 0.0}, //Left
                Point{ x: 5.0, y: -10.0, z: 4.0} //Pointy Bottom
            ]
        ).add_light_effects(0.0, 1.0, 1.3)),

        Box::new(Plane::new(
            [
                Point{ x: 5.0, y: -5.0, z: 0.0},  //Right
                Point{ x: 5.0, y: -10.0, z: 0.0}, //Left
                Point{ x: 6.0, y: -10.0, z: 4.0} //Pointy top
            ]
        ).add_light_effects(0.0, 1.0, 1.3)),

        Box::new(Plane::new(
            [
                Point{ x: 5.0, y: -5.0, z: 0.0},  //Right
                Point{ x: 6.0, y: -10.0, z: 4.0}, //Pointy top
                Point{ x: 5.0, y: -10.0, z: 4.0} //Pointy Bottom
            ]
        ).add_light_effects(0.0, 1.0, 1.3)),

        Box::new(Plane::new(
            [
                Point{ x: 5.0, y: -10.0, z: 0.0}, //Left
                Point{ x: 6.0, y: -10.0, z: 4.0}, //Pointy top
                Point{ x: 5.0, y: -10.0, z: 4.0} //Pointy Bottom
            ]
        ).add_light_effects(0.0, 0.9, 1.4)),

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
        )),
        Box::new(Plane::new(
            [
                Point{ x: 5.0, y: 0.0, z: 0.0},
                Point{ x: 0.0, y: 5.0, z: 0.0},
                Point{ x: 0.0, y: 0.0, z: -5.0}
            ]
        )),

        Box::new(Plane::new(
            [
                Point{ x: 0.0, y: -5.0, z: 0.0},
                Point{ x: 0.0, y: 5.0, z: 0.0},
                Point{ x: 0.0, y: 0.0, z: -5.0}
            ]
        )),
        Box::new(Plane::new(
            [
                Point{ x: 0.0, y: -5.0, z: 0.0},
                Point{ x: 0.0, y: 5.0, z: 0.0},
                Point{ x: 0.0, y: 0.0, z: 5.0}
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
            origin: Point{ x: 20.0, y: -1.0, z: -4.0 }, color: [0.0, 60.0, 60.0]
        },
        LightSource{
            origin: Point{ x: 5.0, y: -10.0, z: -10.0 }, color: [130.0, 130.0, 130.0]
        }
    ];


    let origin = Point{ x: 2.0, y: -20.0, z: -4.0} * 3.0;
    let target = Point{ x: 2.0, y: 0.0, z: 0.0};

    let viewpoint = ViewPoint{ 
        direction: target - origin,
        origin: origin,
        up: Point{ x: 1.0, y: 0.0, z: 0.0},
        distance: 1.0,
        width: 3.0,
        resolution: (1000, 1000)
    };

    fn no_field(_p: Point) -> Point {
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

    render(&mut surfaces, light_sources, viewpoint, attractor);
}
