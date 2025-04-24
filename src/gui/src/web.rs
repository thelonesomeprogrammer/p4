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

use geometry_msgs::msg::Point;
use std_msgs::msg::String as StrMsg;

use std::borrow::Cow;
use std::collections::vec_deque::VecDeque;
use std::ffi::OsStr;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::SystemTime;

struct RokctState {
    bot: Bot,
    node: RosNode,
    orderlist: Arc<Mutex<VecDeque<StrMsg>>>,
}

#[derive(RustEmbed)]
#[folder = "webdata/"]
struct Asset;

struct RosNode {
    subscription: Arc<Subscription<Point>>,
    publisher: Arc<Publisher<StrMsg>>,
    node: Arc<Node>,
}

#[derive(Clone)]
struct Bot {
    disired: Arc<Mutex<Point>>,
    current: Arc<Mutex<Point>>,
}

#[get("/")]
fn index() -> Option<RawHtml<Cow<'static, [u8]>>> {
    let asset = Asset::get("index.html")?;
    Some(RawHtml(asset.data))
}

#[get("/control")]
fn control(ws: rocket_ws::WebSocket, state: &State<RokctState>) -> rocket_ws::Stream!['static] {
    let list = state.orderlist.clone();
    rocket_ws::Stream! { ws =>
        for await message in ws {
            match message {
                Err(e) => yield format!("{e}").into(),
                Ok(i) => {
                    match i {
                        Message::Text(string) => {(*list.lock().unwrap()).push_back(StrMsg{data:string});},
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
    let point = state.bot.current.clone();

    EventStream! {
        let point = point.clone();
        let mut timer = interval(Duration::from_millis(100));
        loop {
            {
                yield Event::data(format!("x:{};y:{};z:{};",
                        point.lock().unwrap().x,
                        point.lock().unwrap().y,
                        point.lock().unwrap().z,
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
    let disired = Arc::new(Mutex::new(Point {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    }));
    let current = Arc::new(Mutex::new(Point {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    }));

    let current_c = current.clone();
    let bot = Bot {
        disired: disired.clone(),
        current: current.clone(),
    };

    let context = rclrs::Context::new(std::env::args()).expect("context");
    let node = Node::new(&context, "web").expect("1");
    let publisher = node
        .create_publisher("out_topic", rclrs::QOS_PROFILE_DEFAULT)
        .expect("pub");
    let subscription = {
        node.create_subscription("in_topic", rclrs::QOS_PROFILE_DEFAULT, move |msg: Point| {
            *current_c.lock().unwrap() = msg.clone();
        })
        .expect("sub")
    };
    let node = RosNode {
        node,
        subscription,
        publisher,
    };

    let orderlist: Arc<Mutex<VecDeque<StrMsg>>> = Arc::new(Mutex::new(VecDeque::new()));
    let spin_node = node.node.clone();

    std::thread::spawn(move || rclrs::spin(spin_node.clone()).unwrap());
    rocket::build()
        .manage(RokctState {
            node,
            bot,
            orderlist,
        })
        .mount("/", routes![index, dist, control, stream])
}
