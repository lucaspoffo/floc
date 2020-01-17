use kiss3d::light::Light;
use kiss3d::window::Window;
use nalgebra::Point2;
use floc;

fn main() {
    let mut window = Window::new("Steering");
    window.set_light(Light::StickToCamera);

    let mut sm = floc::SteeringManager::new();

    for i in 0..50 {
        sm.add_agent(floc::Agent::new(&mut window, i));
    }

    while window.render() {
        sm.update(&(window.width() as f32), &(window.height() as f32));
        match window.cursor_pos() {
            Some((x, y)) => { 
                let ww = (window.width() / 2) as f32;
                let wh = (window.height() / 2) as f32;
                sm.update_target(Point2::new(x as f32 - ww,-y as f32 + wh));
            },
            _ => ()
        }
        
    }
}
