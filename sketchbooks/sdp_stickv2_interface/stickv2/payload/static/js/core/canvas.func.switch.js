/*
    判断是否点击相同功能
*/
function isClickSameFunc(type_id){
    if (type_id != func_id) {
        return false;
    }else{    
        return true;
    }
}

/*模板方法*/
function baseTemplate(ts) {
    //停止上一次的流数据
    stopLoadStream()
    //去除音频canvas
    remove_fft_canvas()
    //初始化拍照canvas
    camera_init(canvas);
    //文件上传初始化
    uploading_init();
    //canvas区域显示
    $(".stream").show();
    //选中按钮高亮
    btnActive(ts)
    //重置
    clearDynamicFuncArea();
    function_reset()
}

function loadStreamTemplate(ts) {
    baseTemplate(ts);
    loadStream();
}

function drawRectTemplate(ts) {
    baseTemplate(ts);
    drawRect();
}

function loadStreamContext(){
    $(".stream").show();
    remove_fft_canvas();
}

function btnActive(ts) {
    $(".btn-active").removeClass("btn-active")
    $(ts).addClass("btn-active")
}

function clearDynamicFuncArea() {
    clearRect();
    offCanvasEvent();
    $("#dynamic_func_area").empty();
}
/*end*/ 


//功能变量重置
function function_reset() {
    //fr
    face_recognition_type_count = 0;
    face_recognition_type_val = "face_0";
    face_recognition_current_id = 0;
    face_recognition_btn_train_flag = 0;
    //otc
    classifier_online_type_count = 0;
    classifier_online_type_val = 'class_0';
    classifier_online_current_id = 0;
   
    //lab
    color_tracker_btn_model_flag = 0;
    color_tracker_l1 = 0;
    color_tracker_l2 = 0;
    color_tracker_a1 = 0; 
    color_tracker_a2 = 0;
    color_tracker_b1 = 0;
    color_tracker_b2 = 0;
    line_tracker_btn_model_flag = 0;
    line_tracker_l1 = 0;
    line_tracker_l2 = 0;
    line_tracker_a1 = 0; 
    line_tracker_a2 = 0;
    line_tracker_b1 = 0;
    line_tracker_b2 = 0;
    
    //画框
    draw_rect_flag = 0;
    x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    //按钮区域
    console_clear()
    //重置画面渲染数据
    stream_res = null;
}



























