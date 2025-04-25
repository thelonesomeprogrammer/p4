#[macro_use]
extern crate rocket;

use serde::{Deserialize, Serialize};

use rocket::http::ContentType;
use rocket::response::content::RawHtml;
use rocket::response::stream::{Event, EventStream};
use rocket::serde::json::Json;
use rocket::tokio;
use rocket::tokio::time::{interval, Duration};
use rocket::State;

use rocket_ws;
use rocket_ws::Message;

use rust_embed::RustEmbed;

use rclrs::{Node, Publisher, Subscription};

use geometry_msgs::msg::{Point, Pose, Quaternion};
use std_msgs::msg::Bool as Rbool;

use std::borrow::Cow;
use std::ffi::OsStr;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::SystemTime;

struct RokctState {
    bot: Bot,
    node: RosNode,
}

#[derive(RustEmbed)]
#[folder = "webdata/"]
struct Asset;

struct RosNode {
    publisher: Arc<Publisher<Point>>,
    pos_subscription: Arc<Subscription<Pose>>,
    link_subscription: Arc<Subscription<Rbool>>,
    node: Arc<Node>,
}

#[derive(Clone)]
struct Bot {
    current: Arc<Mutex<Pose>>,
    cf_connected: Arc<Mutex<bool>>,
}

#[get("/")]
fn index() -> Option<RawHtml<Cow<'static, [u8]>>> {
    let asset = Asset::get("index.html")?;
    Some(RawHtml(asset.data))
}

#[get("/control")]
fn control(ws: rocket_ws::WebSocket, state: &State<RokctState>) -> rocket_ws::Stream!['static] {
    let publisher = state.node.publisher.clone();
    rocket_ws::Stream! { ws =>
        for await message in ws {
            match message {
                Err(e) => yield format!("{e}").into(),
                Ok(i) => {
                    match i {
                        Message::Text(string) => {
                            let mut data = string.split(';');
                            let x = data.next().unwrap_or("0").parse::<f64>().unwrap_or(0.0);
                            let y = data.next().unwrap_or("0").parse::<f64>().unwrap_or(0.0);
                            let z = data.next().unwrap_or("0").parse::<f64>().unwrap_or(0.0);
                            publisher
                                .publish(&Point {
                                    x,
                                    y,
                                    z,
                                })
                                .expect("pub");

                        },
                        Message::Binary(_data) => {},
                        Message::Frame(_frame) => {},
                        Message::Close(_opt)   => {},
                        Message::Ping(_data)   => {},
                        Message::Pong(_data)   => {},
                    }
                    yield "ok".to_string().into();
                }
            }
        }
    }
}

#[get("/stream")]
fn stream(state: &State<RokctState>) -> EventStream![] {
    let pose = state.bot.current.clone();
    let link = state.bot.cf_connected.clone();

    EventStream! {
        let link = link.clone();
        let pose = pose.clone();
        let mut timer = interval(Duration::from_millis(100));
        loop {
            {
                let ori = pose.lock().unwrap().orientation.clone();
                let rpy = [
                    ori.x.atan2(ori.y),
                    ori.y.atan2(ori.z),
                    ori.z.atan2(ori.w),
                ];
                yield Event::data(format!("x:{};y:{};z:{};r:{};p:{};y:{};link:{}",
                    pose.lock().unwrap().position.x,
                    pose.lock().unwrap().position.y,
                    pose.lock().unwrap().position.z,
                    rpy[0],
                    rpy[1],
                    rpy[2],
                    link.lock().unwrap().clone(),
                        ));
            }
            timer.tick().await;
        }
    }
}

#[get("/dist/<file..>")]
fn dist(file: PathBuf) -> Option<(ContentType, Cow<'static, [u8]>)> {
    let filename = file.display().to_string();
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
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        orientation: Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        },
    }));
    let link = Arc::new(Mutex::new(false));

    let current_c = current.clone();
    let bot = Bot {
        current: current.clone(),
        cf_connected: link.clone(),
    };

    let context = rclrs::Context::new(std::env::args()).expect("context");
    let node = Node::new(&context, "web").expect("1");
    let publisher = node
        .create_publisher("cf_command", rclrs::QOS_PROFILE_DEFAULT)
        .expect("pub");
    let pos_subscription = {
        node.create_subscription("cf_pos", rclrs::QOS_PROFILE_DEFAULT, move |msg: Pose| {
            *current_c.lock().unwrap() = msg.clone();
        })
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
        .mount("/", routes![index, dist, control, stream])
}
