#[macro_use]
extern crate rocket;

use serde::{Deserialize, Serialize};

use rocket::http::ContentType;
use rocket::response::content::RawHtml;
use rocket::tokio;
use rocket::tokio::time::{interval, Duration};
use rocket::State;

use rust_embed::RustEmbed;

use rclrs::{Node, Publisher, Subscription};

use geometry_msgs::msg::{Point, Pose, PoseStamped, Quaternion};
use std_msgs::msg::Bool as Rbool;

use std::borrow::Cow;
use std::ffi::OsStr;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::SystemTime;

use rocket::http::Header;
use rocket::{
    fairing::{Fairing, Info, Kind},
    http::Method,
    Request, Response,
};

pub struct CORS;

#[rocket::async_trait]
impl Fairing for CORS {
    fn info(&self) -> Info {
        Info {
            name: "Add CORS headers",
            kind: Kind::Response,
        }
    }

    async fn on_response<'r>(&self, request: &'r Request<'_>, response: &mut Response<'r>) {
        response.set_header(Header::new(
            "Access-Control-Allow-Origin",
            "http://localhost:3000",
        )); // or your dev server's port
        response.set_header(Header::new(
            "Access-Control-Allow-Methods",
            "GET, POST, PUT, DELETE, OPTIONS",
        ));
        response.set_header(Header::new("Access-Control-Allow-Headers", "Content-Type"));

        // Needed for preflight OPTIONS requests
        if request.method() == Method::Options {
            response.set_status(rocket::http::Status::Ok);
        }
    }
}

struct RokctState {
    bot: Bot,
    node: RosNode,
}

#[derive(RustEmbed)]
#[folder = "webdata/"]
struct Asset;

struct RosNode {
    publisher: Arc<Publisher<Point>>,
    pos_subscription: Arc<Subscription<PoseStamped>>,
    link_subscription: Arc<Subscription<Rbool>>,
    node: Arc<Node>,
}

#[derive(Clone)]
struct Bot {
    current: Arc<Mutex<Pose>>,
    cf_connected: Arc<Mutex<bool>>,
    resp: Arc<Mutex<String>>,
}

#[get("/")]
fn index() -> Option<RawHtml<Cow<'static, [u8]>>> {
    let asset = Asset::get("index.html")?;
    Some(RawHtml(asset.data))
}

#[post("/api/control", data = "<point>")]
fn control(point: String, state: &State<RokctState>) {
    let mut data = point.split(';').filter_map(|s| s.split(':').nth(1));
    let x = data.next().unwrap_or("0").parse::<f64>().unwrap_or(0.0);
    let y = data.next().unwrap_or("0").parse::<f64>().unwrap_or(0.0);
    let z = data.next().unwrap_or("0").parse::<f64>().unwrap_or(0.0);
    state.node.publisher.clone().publish(&Point { x, y, z });
}

#[get("/api/streams")]
fn stream(state: &State<RokctState>) -> String {
    let pose = state.bot.current.clone().lock().unwrap().clone();
    let link = state.bot.cf_connected.clone().lock().unwrap().clone();
    let q = pose.orientation.clone();
    // roll (x-axis rotation)
    let sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    let cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    let roll = sinr_cosp.atan2(cosr_cosp);

    // pitch (y-axis rotation)
    let sinp = (1.0 + 2.0 * (q.w * q.y - q.x * q.z)).sqrt();
    let cosp = (1.0 - 2.0 * (q.w * q.y - q.x * q.z)).sqrt();
    let pitch = 2.0 * sinp.atan2(cosp) - std::f64::consts::PI / 2.0;

    // yaw (z-axis rotation)
    let siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    let cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    let yaw = siny_cosp.atan2(cosy_cosp);

    let rpy = [
        -pitch * 180.0 / std::f64::consts::PI,
        roll * 180.0 / std::f64::consts::PI,
        yaw * 180.0 / std::f64::consts::PI,
    ];
    return format!(
        "x:{};y:{};z:{};r:{};p:{};y:{};link:{}",
        pose.position.x, pose.position.y, pose.position.z, rpy[0], rpy[1], rpy[2], link,
    );
}

#[get("/static/<file..>")]
fn dist(file: PathBuf) -> Option<(ContentType, Cow<'static, [u8]>)> {
    let filename = "static/".to_string() + &file.display().to_string();
    let asset = Asset::get(&filename)?;
    let content_type = file
        .extension()
        .and_then(OsStr::to_str)
        .and_then(ContentType::from_extension)
        .unwrap_or(ContentType::Bytes);

    Some((content_type, asset.data))
}

#[launch]
async fn rocket() -> _ {
    let current = Arc::new(Mutex::new(Pose {
        position: Point {
            x: 0.1,
            y: 0.5,
            z: 1.0,
        },
        orientation: Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        },
    }));
    let resp = Arc::new(Mutex::new("".to_string()));
    let link = Arc::new(Mutex::new(false));
    let link_c = link.clone();

    let current_c = current.clone();
    let current_c2 = current.clone();
    let bot = Bot {
        current,
        cf_connected: link.clone(),
        resp: resp.clone(),
    };

    let context = rclrs::Context::new(std::env::args()).expect("context");
    let node = Node::new(&context, "web").expect("1");
    let publisher = node
        .create_publisher("cf_command", rclrs::QOS_PROFILE_DEFAULT)
        .expect("pub");
    let pos_subscription = {
        node.create_subscription(
            "/vicon/Group466CF/Group466CF/pose",
            rclrs::QOS_PROFILE_DEFAULT,
            move |msg: PoseStamped| {
                *current_c.lock().unwrap() = msg.pose.clone();
            },
        )
        .expect("sub")
    };
    let link_subscription = {
        node.create_subscription("cf_link", rclrs::QOS_PROFILE_DEFAULT, move |msg: Rbool| {
            *link.lock().unwrap() = msg.data.clone();
        })
        .expect("sub")
    };
    let node = RosNode {
        node,
        publisher,
        pos_subscription,
        link_subscription,
    };

    let spin_node = node.node.clone();

    std::thread::spawn(move || rclrs::spin(spin_node.clone()).unwrap());
    rocket::build()
        .manage(RokctState { node, bot })
        .attach(CORS)
        .mount("/", routes![index, dist, control, stream])
}
