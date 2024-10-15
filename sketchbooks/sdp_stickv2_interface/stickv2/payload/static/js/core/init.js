//canvas 
var img= document.getElementById("img");
var canvas = document.getElementById('canvas');
var ctx = canvas.getContext('2d');

//render
var stream_type, stream_x1, stream_y1, stream_x2, stream_y2, stream_w, 
    stream_h, stream_r, stream_color, stream_data, stream_thickness;
var stream_array_x,stream_array_y;
var stream_res;

//请求数据
var ajax_interval;

// bin_function 
var func_id;
var func_name;
var boot_name;
var current_func_id ;
var func_init = 0;
var func_list = ["object_recognition","code_detector","camera_stream","color_tracker"
                ,"lane_line_tracker","target_tracker","motion_tracker","online_classifier"
                ,"face_recognition","face_detector","shape_detector","shape_matching","audio_fft"];
var toggle_defalut_func_list = 0;

//wifi
var ssid,password;


//post to service
var data;
var roi_payload;

function init(){
    book_func_init();  
    menu_init();
    gallery_init();
    camera_init(canvas);
    file_upload_init();  
    canvasInfoInit();
}