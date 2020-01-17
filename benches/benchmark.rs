use criterion::{criterion_group, criterion_main, Criterion};
use kiss3d::light::Light;
use kiss3d::window::Window;
use floc;

fn criterion_benchmark(c: &mut Criterion) {
  let mut window = Window::new("Steering");
    window.set_light(Light::StickToCamera);

    let mut sm = floc::SteeringManager::new();

    for i in 0..50 {
        sm.add_agent(floc::Agent::new(&mut window, i));
    }

    c.bench_function("steering", |b| b.iter(|| for _i in 0..10 { sm.update(&(window.width() as f32), &(window.height() as f32)); }));
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
