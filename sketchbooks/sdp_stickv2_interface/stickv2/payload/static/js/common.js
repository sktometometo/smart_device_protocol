
var core_js_array = [
    "init.js",
    "message.box.js",
    "canvas.info.js",
    "func.menu.js",
    "func.result.js",
    "gallery.js",
    "camera.js",
    "color.conversion.js",
    "file.upload.js",
    "uploading.js",
    "wifi.js",
    "boot.func.js",
    "render.js",
    "menu.js",
    "post.server.js",
    "canvas.draw.js",
    "canvas.func.loading.js",
    "canvas.func.switch.js",
    "start.up.js"
]
var bin_js_array = [
    "common.utils.js",
    "online_classifier.js",
    "face_recognition.js",
    "face_detector.js",
    "color_tracker.js",
    "lane_line_tracker.js",
    "target_tracker.js",
    "motion_tracker.js",
    "shape_detector.js",
    "shape_matching.js",
    "camera_stream.js",
    "code_detector.js",
    "object_recognition.js",
    "fast_fourier_transform.js"
]
const FLASK_PREFIX = "/static/";
const CORE_JS_PREFIX = "./js/core/";
const BIN_JS_PREFIX = "./js/bin/";
function load_js(){
    for(let bin_js of bin_js_array){
        let script = document.createElement('script');
        script.type = "text/javascript";
        script.src =  FLASK_PREFIX + BIN_JS_PREFIX + bin_js;
        document.getElementsByTagName("head")[0].appendChild(script);
    }
    for(let core_js of core_js_array){
        let script = document.createElement('script');
        script.type = "text/javascript";
        script.src =  FLASK_PREFIX + CORE_JS_PREFIX + core_js;
        document.getElementsByTagName("head")[0].appendChild(script);
    }
}

$(function(){
    load_js();
})