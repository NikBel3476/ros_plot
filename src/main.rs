use std::f64::consts::PI;

use rosrust::{ros_info, Publisher};
use rosrust_msg::{geometry_msgs, nav_msgs::Odometry};

const NUMBER_OF_POINTS: usize = 150;
const MAX_LINEAR_VELOCIY: f64 = 1.5;
const MAX_ANGULAR_VELOCITY: f64 = 1.5;
const THETA_START: f64 = 0.0;

static mut LINEAR_VEL_REAL: f64 = 0.0;
static mut ANGULAR_VEL_REAL: f64 = 0.0;

fn main() {
    env_logger::init();

    // Initialize node
    rosrust::init("plot_test");

    let cmd_vel_pub: Publisher<geometry_msgs::Twist> = rosrust::publish("cmd_vel", 10).unwrap();
    cmd_vel_pub.wait_for_subscribers(None).unwrap();
    let pub_theor_trajectory = rosrust::publish("theor_trajectory", 10).unwrap();
    pub_theor_trajectory.wait_for_subscribers(None).unwrap();
    let pub_theor_lin_vel = rosrust::publish("theor_lin_vel", 10).unwrap();
    pub_theor_lin_vel.wait_for_subscribers(None).unwrap();
    let pub_theor_ang_vel = rosrust::publish("theor_ang_vel", 10).unwrap();
    pub_theor_ang_vel.wait_for_subscribers(None).unwrap();
    let pub_real_lin_vel = rosrust::publish("real_lin_vel", 10).unwrap();
    pub_real_lin_vel.wait_for_subscribers(None).unwrap();
    let pub_real_ang_vel = rosrust::publish("real_ang_vel", 10).unwrap();
    pub_real_ang_vel.wait_for_subscribers(None).unwrap();

    let odom_sub = rosrust::subscribe("odom", 10, |odometry: Odometry| unsafe {
        LINEAR_VEL_REAL = odometry.twist.twist.linear.x;
        ANGULAR_VEL_REAL = odometry.twist.twist.angular.z;
    })
    .unwrap();

    let t_points = calculate_t_points();
    let d_points = calculate_derivative_points(&t_points);
    let points_of_trajectory = calculate_points_of_trajectory(&t_points);
    // ros_info!("{:#?}", points_of_trajectory);

    let rate = rosrust::rate(100.0);
    // for point in &points_of_trajectory {
    //     pub_theor_trajectory
    //         .send(geometry_msgs::Vector3 {
    //             x: point.0,
    //             y: point.1,
    //             z: 0.0,
    //         })
    //         .unwrap();
    //     rate.sleep();
    // }

    for point in &d_points {
        pub_theor_trajectory
            .send(geometry_msgs::Vector3 {
                x: point.0,
                y: point.1,
                z: 0.0,
            })
            .unwrap();
        rate.sleep();
    }

    let dt = 0.25;
    let (v, w) = calculate_velocities(&points_of_trajectory, dt);

    let max_lin_velocity = v.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let max_ang_velocity = w.iter().copied().fold(f64::NEG_INFINITY, f64::max);

    let coeff_safety = 1.0;
    let k1 = MAX_LINEAR_VELOCIY / max_lin_velocity * coeff_safety;
    let k2 = MAX_ANGULAR_VELOCITY / max_ang_velocity * coeff_safety;
    let k = k1.min(k2).clamp(0.1, coeff_safety);

    for (i, velocity) in v.iter().enumerate() {
        pub_theor_lin_vel
            .send(geometry_msgs::Vector3 {
                x: i as f64,
                y: velocity.clone(),
                z: 0.0,
            })
            .unwrap();
        rate.sleep();
    }

    for (i, velocity) in w.iter().enumerate() {
        pub_theor_ang_vel
            .send(geometry_msgs::Vector3 {
                x: i as f64,
                y: velocity.clone(),
                z: 0.0,
            })
            .unwrap();
        rate.sleep();
    }

    let duration = rosrust::Duration::from_nanos((dt * 1E9) as i64);
    for (i, velocity) in v.iter().zip(&w).enumerate() {
        unsafe {
            pub_real_lin_vel
                .send(geometry_msgs::Vector3 {
                    x: i as f64,
                    y: LINEAR_VEL_REAL,
                    z: 0.0,
                })
                .unwrap();
            pub_real_ang_vel
                .send(geometry_msgs::Vector3 {
                    x: i as f64,
                    y: ANGULAR_VEL_REAL,
                    z: 0.0,
                })
                .unwrap();
        }
        cmd_vel_pub
            .send(geometry_msgs::Twist {
                linear: geometry_msgs::Vector3 {
                    x: velocity.0.clone(),
                    y: 0.0,
                    z: 0.0,
                },
                angular: geometry_msgs::Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: velocity.1.clone(),
                },
            })
            .unwrap();
        // cmd_vel_rate.sleep();
        rosrust::sleep(duration);
    }

    cmd_vel_pub.send(geometry_msgs::Twist {
        linear: geometry_msgs::Vector3::default(),
        angular: geometry_msgs::Vector3::default()
    }).unwrap();
}

fn calculate_t_points() -> Vec<f64> {
    let mut t_points = vec![0.0];
    let step = PI / (NUMBER_OF_POINTS as f64);
    let mut current_value = step;
    for _ in 1..NUMBER_OF_POINTS {
        t_points.push(current_value);
        current_value += step;
    }

    // sum of sin
    // let mut t_points = vec![];
    // let a1= 23.55;
    // let b1= 0.2186;
    // let c1= 1.579;
    // let a2= 23.25;
    // let b2= 0.2487;
    // let c2= 4.755 ;
    // let a3= 0.1469;
    // let b3= 1.11;
    // let c3= 4.841;
    // let a4= 0.1867;
    // let b4= 3.519;
    // let c4= -1.52;

    // let end_t = 8.0;
    // for i in 0..NUMBER_OF_POINTS {
    //     let s = i as f64 / NUMBER_OF_POINTS as f64 * end_t;
    //     t_points.push(a1*(b1*s+c1).sin() + a2*(b2*s+c2).sin() + a3*(b3*s+c3).sin() + a4*(b4*s+c4).sin());
    // }

    // 2*PI*sin^2(t)
    // let mut t_points = vec![];
    // let end_t = PI;
    // for i in 0..NUMBER_OF_POINTS {
    //     let s = i as f64 / NUMBER_OF_POINTS as f64 * end_t;
    //     t_points.push(2.0 * PI * (s + i as f64 * PI / 2.0).sin().powi(2));
    // }

    t_points
}

fn calculate_derivative_points(t_points: &[f64]) -> Vec<(f64, f64)> {
    let mut d_points = vec![];
    for point in t_points {
        d_points.push((
            4.0 * (2.0 * point).sin() * (2.0 * point).cos(),
            -3.0 * (3.0 * point).sin()
        ))
    }
    d_points
}

fn calculate_points_of_trajectory(t_points: &[f64]) -> Vec<(f64, f64)> {
    let k_tr = 1.0;
    let shift = (0.0, -1.0);
    let mut trajectory_points = vec![];
    for point in t_points {
        // variant 14
        trajectory_points.push((
            ((2.0 * point).sin().powi(2) + shift.0) * k_tr,
            ((3.0 * point).cos() + shift.1) * k_tr,
        ));
    }
    trajectory_points
}

fn calculate_velocities(points: &[(f64, f64)], dt: f64) -> (Vec<f64>, Vec<f64>) {
    let mut dl = vec![];
    let mut dw = vec![];
    let mut theta = vec![];
    let mut v = vec![];
    let mut w = vec![];
    for i in 0..NUMBER_OF_POINTS - 1 {
        let dx = points[i + 1].0 - points[i].0;
        let dy = points[i + 1].1 - points[i].1;
        dl.push((dx.powi(2) + dy.powi(2)).sqrt());
        theta.push(dy.atan2(dx));
        if theta[i] < 0.0 {
            theta[i] += 2.0 * PI;
        }

        dw.push(if i == 0 {
            theta[i] - THETA_START
        } else {
            theta[i] - theta[i - 1]
        });

        dw[i] = remap_angle(dw[i]);

        v.push(dl[i] / dt);
        w.push(dw[i] / dt);
    }

    (v, w)
}

fn remap_angle(value: f64) -> f64 {
    if value > PI {
        return value - 2.0 * PI;
    }
    if value < -PI {
        return value + 2.0 * PI;
    }
    return value;
}
