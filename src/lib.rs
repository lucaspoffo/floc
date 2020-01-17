use kiss3d::light::Light;
use kiss3d::{window::Window, scene::PlanarSceneNode};
use nalgebra::{Vector2, Point2, Translation2};
use rand::{Rng, thread_rng};

const MAX_SEPARATION: f32 = 16.0;
const MAX_ALIGNMENT: f32 = 32.0;
const MAX_COHESION: f32 = 16.0;
const MAX_VELOCITY: f32 = 6.0;
const MAX_FORCE: f32 = 0.05;

const SEPARITION_WEIGHT: f32 = 4.0;
const ALIGNMENT_WEIGHT: f32 = 3.0;
const COHESION_WEIGHT: f32 = 2.0;

trait Truncate {
  fn truncate(&mut self, length: f32);
}

impl Truncate for Vector2<f32> {
  fn truncate(&mut self, length: f32) {
      if nalgebra::Matrix::magnitude(self) > length {
          let n = nalgebra::Matrix::normalize(self) * length;
          self.x = n.x;
          self.y = n.y;
      }
  }
}

#[derive(Clone)]
pub struct Agent {
  id: u32,
  position: Point2<f32>,
  rotation: f32,
  velocity: Vector2<f32>,
  scene: PlanarSceneNode,
  target: Point2<f32>,
}

impl Agent {
  pub fn new(window: &mut Window, id: u32) -> Agent {
      let mut rng = thread_rng();
      let x = rng.gen_range(-400, 400) as f32;    
      let y = rng.gen_range(-400, 400) as f32;    
      let vx = rng.gen_range(-MAX_VELOCITY, MAX_VELOCITY) as f32;    
      let vy = rng.gen_range(-MAX_VELOCITY, MAX_VELOCITY) as f32;
      let mut velocity = Vector2::new(vx, vy);
      velocity.truncate(MAX_VELOCITY);

      let mut scene = window.add_rectangle(4.0, 4.0);
      scene.set_local_translation(Translation2::new(x, y));
      Agent { scene, rotation: 0.0, position: Point2::new(x, y), velocity, target: Point2::new(0.0, 0.0), id }
  }

  fn wrap(&mut self, width: &f32, height: &f32) {
      if self.position.x < -width / 2.0 {
          self.position.x = width / 2.0;
      } else if self.position.x > width / 2.0 {
          self.position.x = -width / 2.0;
      } else if self.position.y < -height / 2.0 {
          self.position.y = height / 2.0;
      } else if self.position.y > height / 2.0 {
          self.position.y = -height / 2.0;
      }
  }
}

#[derive(Clone)]
pub struct SteeringManager {
  agents: Vec<Agent>,
}

impl SteeringManager {
  pub fn new() -> SteeringManager {
      SteeringManager { agents: vec![] }
  }

  pub fn add_agent(&mut self, agent: Agent) {
      self.agents.push(agent);
  }

  pub fn update(&mut self, width: &f32, height: &f32) {
      let agents = self.agents.clone();
      self.agents.iter_mut().for_each(|a| {
          let mut steer: Vector2<f32> = Vector2::new(0.0, 0.0);
          steer += SteeringManager::separation_force(a, &agents) * SEPARITION_WEIGHT;
          steer += SteeringManager::alignment_force(a, &agents) * ALIGNMENT_WEIGHT;
          steer += SteeringManager::cohesion_force(a, &agents) * COHESION_WEIGHT;
          steer.truncate(MAX_FORCE);
          a.velocity += steer;
          a.velocity.truncate(MAX_VELOCITY);
          a.position += a.velocity;
          a.wrap(&width, &height);
          a.scene.set_local_translation(Translation2::new(a.position.x, a.position.y));
      }); 
  }

  pub fn update_target(&mut self, target: Point2<f32>) {
      self.agents.iter_mut().for_each(|a| {
          a.target = target;
      })
  }

  fn separation_force(agent: &Agent, neighbors: &Vec<Agent>) -> Vector2<f32> {
      let mut force = Vector2::new(0.0, 0.0);
      let mut neighbor_count = 0;
      neighbors.iter().filter(|n| {
        let d = nalgebra::distance(&n.position, &agent.position);
        d < MAX_SEPARATION && n.id != agent.id
      }).for_each(|n| {
        force += n.position - agent.position;
        neighbor_count += 1;
      });

      force *= -1.0;
      if neighbor_count == 0 || force == Vector2::new(0.0, 0.0) {
          return Vector2::new(0.0, 0.0);
      }
      let desired_velocity = nalgebra::Matrix::normalize(&force) * MAX_VELOCITY;
      desired_velocity - agent.velocity
  }

  fn alignment_force(agent: &Agent, neighbors: &Vec<Agent>) -> Vector2<f32> {
      let mut force = Vector2::new(0.0, 0.0);
      for a in neighbors {
          let d = nalgebra::distance(&a.position, &agent.position);
          if d < MAX_ALIGNMENT && a.id != agent.id {
              force += a.velocity;
          }
      }
      if force == Vector2::new(0.0, 0.0) {
          return force;
      }
      let desired_velocity = nalgebra::Matrix::normalize(&force) * MAX_VELOCITY;
      desired_velocity - agent.velocity
  }

  fn cohesion_force(agent: &Agent, neighbors: &Vec<Agent>) -> Vector2<f32> {
      let mut force = Vector2::new(0.0, 0.0);
      let mut neighbor_count = 0;
      for a in neighbors {
          let d = nalgebra::distance(&a.position, &agent.position);
          if d < MAX_COHESION && a.id != agent.id {
              force += Vector2::new(a.position.x, a.position.y);
              neighbor_count += 1;
          }
      }
      if neighbor_count == 0 || force == Vector2::new(0.0, 0.0) {
          return force;
      }
      force /= neighbor_count as f32;
      force -= Vector2::new(agent.position.x, agent.position.y);
      let desired_velocity = nalgebra::Matrix::normalize(&force) * MAX_VELOCITY;
      desired_velocity - agent.velocity
  }

  fn seek_force(agent: &Agent) -> Vector2<f32> {
      let force = agent.target - agent.position;
      let desired_velocity = nalgebra::Matrix::normalize(&force) * MAX_VELOCITY;
      desired_velocity - agent.velocity
  }
}
