use once_cell::sync::Lazy;
use rosrust::{ros_info, Publisher};
use rosrust_msg::{geometry_msgs, geometry_msgs::PoseWithCovariance, nav_msgs::Odometry};
use std::f64::consts::PI;
use std::sync::Mutex;

const NUMBER_OF_POINTS: usize = 125;
const VELOCITY_COEFFICIENT: f64 = 1.0;
const SHIFT: (f64, f64) = (0.0, -1.0);
const K_D: f64 = 2.0;
const K_P: f64 = 1.0;
const TIME_COEFFICIENT: f64 = 0.25;
const DT: f64 = 0.05;

static mut LINEAR_VEL_REAL: f64 = 0.0;
static mut ANGULAR_VEL_REAL: f64 = 0.0;
static CURRENT_POSE: Lazy<Mutex<geometry_msgs::PoseWithCovariance>> =
    Lazy::new(|| Mutex::new(Default::default()));
static CURRENT_TWIST: Lazy<Mutex<geometry_msgs::TwistWithCovariance>> =
    Lazy::new(|| Mutex::new(Default::default()));

const USE_PID: bool = true;

fn main() {
    match USE_PID {
        true => run_with_pid(),
        false => run_without_feedback(),
    }
}

fn run_without_feedback() {
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

    let _odom_sub = rosrust::subscribe("odom", 10, |odometry: Odometry| unsafe {
        LINEAR_VEL_REAL = odometry.twist.twist.linear.x;
        ANGULAR_VEL_REAL = odometry.twist.twist.angular.z;
        let mut pose = CURRENT_POSE.lock().unwrap();
        *pose = odometry.pose;
        let mut twist = CURRENT_TWIST.lock().unwrap();
        *twist = odometry.twist;
    })
    .unwrap();

    let t_points = calculate_t_points();
    // let points_of_trajectory = rotate_points(&calculate_points_of_trajectory(&t_points), PI / 4.0);
    let points_of_trajectory = calculate_points_of_trajectory(&t_points);

    // Publish points of trajectory to plot
    let rate = rosrust::rate(100.0);
    for point in &points_of_trajectory {
        pub_theor_trajectory
            .send(geometry_msgs::Vector3 {
                x: point.0,
                y: point.1,
                z: 0.0,
            })
            .unwrap();
        rate.sleep();
    }

    let (v, w) = calculate_velocities(&points_of_trajectory, DT);

    let max_lin_velocity = v
        .iter()
        .copied()
        .map(|velocity| velocity.abs())
        .fold(f64::NEG_INFINITY, f64::max);
    let max_ang_velocity = w
        .iter()
        .copied()
        .map(|velocity| velocity.abs())
        .fold(f64::NEG_INFINITY, f64::max);
    ros_info!("MAX_LIN_VELOCITY: {max_lin_velocity}, MAX_ANG_VELOCITY: {max_ang_velocity}");

    // Remap velocities by coefficient
    let linear_velocities: Vec<f64> = v
        .iter()
        .map(|velocity| velocity * VELOCITY_COEFFICIENT)
        .collect();
    let angular_velocities: Vec<f64> = w
        .iter()
        .map(|velocity| velocity * VELOCITY_COEFFICIENT)
        .collect();

    // Publish theoretical linear velocities
    for (i, velocity) in linear_velocities.iter().enumerate() {
        pub_theor_lin_vel
            .send(geometry_msgs::Vector3 {
                x: i as f64,
                y: velocity.clone(),
                z: 0.0,
            })
            .unwrap();
        rate.sleep();
    }

    // Publish theoretical angualr velocities
    for (i, velocity) in angular_velocities.iter().enumerate() {
        pub_theor_ang_vel
            .send(geometry_msgs::Vector3 {
                x: i as f64,
                y: velocity.clone(),
                z: 0.0,
            })
            .unwrap();
        rate.sleep();
    }

    // Rotate robot to remove angular velocity leap on first step
    let dx = points_of_trajectory[1].0 - points_of_trajectory[0].0;
    let dy = points_of_trajectory[1].1 - points_of_trajectory[0].1;
    let angle_to_rotate = dy.atan2(dx);
    let time_to_rotation = angle_to_rotate.abs() * 5.0;
    let duration = rosrust::Duration::from_nanos((time_to_rotation * 1E9 * 1.3) as i64); // increase duration by 30%
    let dw = angle_to_rotate / time_to_rotation;
    cmd_vel_pub
        .send(geometry_msgs::Twist {
            linear: Default::default(),
            angular: geometry_msgs::Vector3 {
                x: 0.0,
                y: 0.0,
                z: dw,
            },
        })
        .unwrap();
    rosrust::sleep(duration);
    cmd_vel_pub.send(geometry_msgs::Twist::default()).unwrap();

    // Publish velocities to robot
    let duration = rosrust::Duration::from_nanos((DT / VELOCITY_COEFFICIENT * 1E9) as i64);
    for (i, velocity) in linear_velocities
        .iter()
        .zip(&angular_velocities)
        .enumerate()
    {
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
        rosrust::sleep(duration);
    }

    cmd_vel_pub
        .send(geometry_msgs::Twist {
            linear: geometry_msgs::Vector3::default(),
            angular: geometry_msgs::Vector3::default(),
        })
        .unwrap();
}

fn run_with_pid() {
    env_logger::init();

    // Initialize node
    rosrust::init("plot_test");

    let cmd_vel_pub: Publisher<geometry_msgs::Twist> = rosrust::publish("cmd_vel", 10).unwrap();
    cmd_vel_pub.wait_for_subscribers(None).unwrap();
    let pub_theor_trajectory = rosrust::publish("theor_trajectory", 10).unwrap();
    pub_theor_trajectory.wait_for_subscribers(None).unwrap();

    let _odom_sub = rosrust::subscribe("odom", 10, |odometry: Odometry| unsafe {
        LINEAR_VEL_REAL = odometry.twist.twist.linear.x;
        ANGULAR_VEL_REAL = odometry.twist.twist.angular.z;
        let mut pose = CURRENT_POSE.lock().unwrap();
        *pose = odometry.pose;
        let mut twist = CURRENT_TWIST.lock().unwrap();
        *twist = odometry.twist;
    })
    .unwrap();

    let t_points = calculate_t_points();
    let points_of_trajectory = calculate_points_of_trajectory(&t_points);

    let rate = rosrust::rate(100.0);
    for point in &points_of_trajectory {
        pub_theor_trajectory
            .send(geometry_msgs::Vector3 {
                x: point.0,
                y: point.1,
                z: 0.0,
            })
            .unwrap();
        rate.sleep();
    }

    let duration = rosrust::Duration::from_nanos((DT * 1E9) as i64);
    let mut w = 0.0;
    let mut a = 0.0;
    let start_time = rosrust::now().seconds();
    let mut time_real_prev = 0.0;
    let mut v_control = 0.0;
    let mut old_v_control = 0.0;
    let mut time_real = 0.0;
    while time_real < PI / TIME_COEFFICIENT {
        cmd_vel_pub
            .send(geometry_msgs::Twist {
                linear: geometry_msgs::Vector3 {
                    x: v_control,
                    y: 0.0,
                    z: 0.0,
                },
                angular: geometry_msgs::Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: w,
                },
            })
            .unwrap();
        rosrust::sleep(duration);

        let twist = (*CURRENT_TWIST).lock().unwrap().clone();
        let robot_velocity = twist.twist.linear.x;
        let pose = (*CURRENT_POSE).lock().unwrap().clone();
        let robot_angle = get_robot_angle();
        (a, w) = velocity_pid(
            &(time_real),
            &pose.pose.position.x,
            &pose.pose.position.y,
            &robot_velocity,
            &robot_angle,
        );
        time_real = rosrust::now().seconds() - start_time;
        let dt_real = time_real - time_real_prev;
        time_real_prev = time_real;

        v_control = old_v_control + a * dt_real;
        old_v_control = v_control;
        ros_info!(
            "a: {:.6}, v: {:.6}, w: {:.6}, dt: {:.6}, ta: {:.6}, x: {:.6}, y: {:.6}, ang: {:.6}",
            a,
            v_control,
            w,
            dt_real,
            time_real,
            pose.pose.position.x,
            pose.pose.position.y,
            robot_angle
        );
    }

    cmd_vel_pub
        .send(geometry_msgs::Twist {
            linear: geometry_msgs::Vector3::default(),
            angular: geometry_msgs::Vector3::default(),
        })
        .unwrap();
}

fn rotate_points(points: &[(f64, f64)], angle: f64) -> Vec<(f64, f64)> {
    points
        .iter()
        .map(|(x, y)| {
            (
                x * angle.cos() - y * angle.sin(),
                x * angle.sin() + y * angle.cos(),
            )
        })
        .collect()
}

fn f(t: &f64) -> f64 {
    (2.0 * t * TIME_COEFFICIENT).sin().powi(2) + SHIFT.0
}

fn g(t: &f64) -> f64 {
    (3.0 * t * TIME_COEFFICIENT).cos() + SHIFT.1
}

fn df(t: &f64) -> f64 {
    TIME_COEFFICIENT * 4.0 * (2.0 * t * TIME_COEFFICIENT).sin() * (2.0 * t * TIME_COEFFICIENT).cos()
}

fn dg(t: &f64) -> f64 {
    TIME_COEFFICIENT * -3.0 * (3.0 * t * TIME_COEFFICIENT).sin()
}

fn ddf(t: &f64) -> f64 {
    TIME_COEFFICIENT.powi(2)
        * 8.0
        * ((2.0 * t * TIME_COEFFICIENT).cos().powi(2) - (2.0 * t * TIME_COEFFICIENT).sin().powi(2))
}

fn ddg(t: &f64) -> f64 {
    TIME_COEFFICIENT.powi(2) * -9.0 * (3.0 * t * TIME_COEFFICIENT).cos()
}

fn velocity_pid(t: &f64, x: &f64, y: &f64, v: &f64, a: &f64) -> (f64, f64) {
    let control_x = ddf(t) + K_D * (df(t) - x_dot(v, a)) + K_P * (f(t) - x);
    let control_y = ddg(t) + K_D * (dg(t) - y_dot(v, a)) + K_P * (g(t) - y);
    let linear_acceleration = a.cos() * control_x + a.sin() * control_y;
    let angular_velocity = -a.sin() / v * control_x + a.cos() / v * control_y;

    (linear_acceleration, angular_velocity)
}

fn x_dot(v: &f64, a: &f64) -> f64 {
    v * a.cos()
}

fn y_dot(v: &f64, a: &f64) -> f64 {
    v * a.sin()
}

fn calculate_t_points() -> Vec<f64> {
    let mut t_points = vec![0.0];
    let step = PI / TIME_COEFFICIENT / (NUMBER_OF_POINTS as f64);
    let mut current_value = step;
    for _ in 1..NUMBER_OF_POINTS {
        t_points.push(current_value);
        current_value += step;
    }

    t_points
}

fn calculate_points_of_trajectory(t_points: &[f64]) -> Vec<(f64, f64)> {
    let k_tr = 1.0;
    let mut trajectory_points = vec![];
    for point in t_points {
        // variant 14
        trajectory_points.push((f(point) * k_tr, g(point) * k_tr));
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
            // theta[i]
            0.0
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

fn get_robot_angle() -> f64 {
    let pose = (*CURRENT_POSE.lock().unwrap()).clone();
    get_angle_from_pose(&pose)
}

fn get_angle_from_pose(pose: &PoseWithCovariance) -> f64 {
    remap_angle(
        (pose.pose.orientation.z.atan2(pose.pose.orientation.w) * 2.0 * 180.0 / PI).to_radians(),
    )
}
