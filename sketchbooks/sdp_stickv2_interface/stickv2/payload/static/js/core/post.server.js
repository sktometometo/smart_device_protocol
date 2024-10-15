
//功能切换
function postUnitV2Func(type_id, type_name) {
    if(ajax_interval !=null){
        clearInterval(ajax_interval);
    }
    func_id = type_id;
    func_name = type_name;
    // if(func_init == 1){
    //     return;
    // }
    var post_data = JSON.stringify({ "type_id": type_id, "type_name": type_name,"args":""});
    if(func_id == '1'){
        post_data =  JSON.stringify({ "type_id": type_id, "type_name": type_name,"args":[yolo_args]})
    }
    createLoading(); 
    $.ajax({
        type: "POST",
        url: "/func",
        contentType: "application/json;charset=UTF-8",
        data: post_data,
        success: function (res) {
            try {
                let doc = JSON.parse(res)
                $(".main").fadeIn('fast');
                $(".canvas-func-title").html(doc.title);
                $(".canvas-func-p").html(doc.description);
                destroyLoading()
            } catch (error) {
                console.log("JSON 解析错误 !")
                destroy_loading = setTimeout("destroyLoading()", 500);
            }
        },
        error: function (res) {
            destroy_loading = setTimeout("destroyLoading()", 500);
        }
    })
}


// face_recognition
function face_recognition_btn_reset_cb() {
    clearDynamicFuncArea();
    loadStream();
    face_recognition_reset();    
    $("#face_recognition_train").css({
        background: '#222831'
    });
    $("#face_recognition_train").html("train");

    face_recognition_btn_add_cb();
    var xmlhttp = new XMLHttpRequest();
    xmlhttp.open("POST", "/data_to_device", true);
    xmlhttp.setRequestHeader("Content-Type", "application/json");
    xmlhttp.send(JSON.stringify({ "operation": "reset" }));

}

function face_recognition_btn_train_cb() {
    let input_val = $("#dynamic_func_area .trainning-input input").val();
    if ($("#dynamic_func_area .trainning-input button").hasClass('train_active') && input_val  != null && input_val  != '') {
        if (face_recognition_btn_train_flag == 0) {
            $("#face_recognition_train").css({
                background: '#ef5773'
            });
            $("#face_recognition_train").html("stop");

            face_recognition_btn_train_flag = 1;
            var xmlhttp = new XMLHttpRequest();
            xmlhttp.open("POST", "/data_to_device", true);
            xmlhttp.setRequestHeader("Content-Type", "application/json");
            xmlhttp.send(JSON.stringify({ "operation": "train", "face_id": face_recognition_current_id, 
                                          "name": face_recognition_type_val }));
        } else {
            $("#face_recognition_train").css({
                background: '#222831'
            });
            $("#face_recognition_train").html("train");
            face_recognition_btn_train_flag = 0;
            var xmlhttp = new XMLHttpRequest();
            xmlhttp.open("POST", "/data_to_device", true);
            xmlhttp.setRequestHeader("Content-Type", "application/json");
            xmlhttp.send(JSON.stringify({ "operation": "stoptrain", "face_id": face_recognition_current_id, 
                                          "name": face_recognition_type_val }));
        }

    } else {
        alert("Please check the training box and input value !")
    }
}

function face_recognition_btn_save_cb() {
    let input_val = $("#dynamic_func_area .trainning-input input").val();
    if ($("#dynamic_func_area .trainning-input button").hasClass('train_active') && input_val  != null && input_val  != '') {
        var xmlhttp = new XMLHttpRequest();
        xmlhttp.open("POST", "/data_to_device", true);
        xmlhttp.setRequestHeader("Content-Type", "application/json");
        xmlhttp.send(JSON.stringify({ "operation": "saverun" }));

        face_recognition_btn_train_flag = 0;
        $("#face_recognition_train").css({
            background: '#222831'
        });
        $("#face_recognition_train").html("train");

    } else {
        alert("Please check the training box and input value !")
    }

}

//classifier_online
function classifier_online_btn_train_cb() {
    let input_val = $("#dynamic_func_area .trainning-input input").val();
    if ($("#dynamic_func_area .trainning-input button").hasClass('train_active') && input_val  != null && input_val  != '') {
        var xmlhttp = new XMLHttpRequest();
        xmlhttp.open("POST", "/data_to_device", true)
        xmlhttp.setRequestHeader("Content-Type", "application/json");
        xmlhttp.send(JSON.stringify({ "operation": "train", "class_id": classifier_online_current_id, 
                                      "class": classifier_online_type_val }));
        draw_rect_flag = 0;
    } else {
        alert("Please check the training box and input value !")
    }
}

function classifier_online_btn_saverun_cb() {
    let input_val = $("#dynamic_func_area .trainning-input input").val();
    if ($("#dynamic_func_area .trainning-input button").hasClass('train_active') && input_val  != null && input_val  != '') {
        var xmlhttp = new XMLHttpRequest();
        xmlhttp.open("POST", "/data_to_device", true);
        xmlhttp.setRequestHeader("Content-Type", "application/json");
        xmlhttp.send(JSON.stringify({ "operation": "saverun" }));
    } else {
        alert("Please check the training box and input value !")
    }

}

function classifier_online_btn_reset_cb() {
    clearDynamicFuncArea();
    loadStream();
    classifier_online_val_reset();
    classifier_online_btn_add_cb();
    var xmlhttp = new XMLHttpRequest();
    xmlhttp.open("POST", "/data_to_device", true);
    xmlhttp.setRequestHeader("Content-Type", "application/json");
    xmlhttp.send(JSON.stringify({ "operation": "reset" }));

}

//motion_tracker
function motion_tracker_btn_demarcate_cb() {
    var xmlhttp = new XMLHttpRequest();
    xmlhttp.open("POST", "/data_to_device", true);
    xmlhttp.setRequestHeader("Content-Type", "application/json");
    xmlhttp.send(JSON.stringify({ "operation": "update" }));

}

//target_tracker
function target_tracker_btn_update_cb() {
    var xmlhttp = new XMLHttpRequest();
    xmlhttp.open("POST", "/data_to_device", true);
    xmlhttp.setRequestHeader("Content-Type", "application/json");
    xmlhttp.send(roi_payload);
    draw_rect_flag = 0;
}


//line_tracker
function line_tracker_lab_roi_update() {
    draw_rect_flag = 0;
    $.ajax({
        type: "post",
        url: "/color_lab",
        contentType: "application/json;charset=UTF-8",
        data: roi_payload,
        dataType:"json",
        success: function (res) {
            if(res  == "ok"){

            }else{

            }
        },
        error: function (res) {
        }
    })

    ajax_interval = setInterval(function(){
        $.ajax({
            type:"post",
            url:"/data_from_device",
            dataType:"json",
            success:function(res){
                if(res != null && res.running == "Lane Line Tracker"){
                   if(res.l_min !=null) {
                        line_tracker_l1 = res.l_min;
                        line_tracker_l2 = res.l_max;
                        line_tracker_a1 = res.a_min;
                        line_tracker_a2 = res.a_max;
                        line_tracker_b1 = res.b_min;
                        line_tracker_b2 = res.b_max;
                        line_tracker_sidlerBarFreeCall("L", line_tracker_l1, line_tracker_l2);
                        line_tracker_sidlerBarFreeCall("A", line_tracker_a1, line_tracker_a2);   
                        line_tracker_sidlerBarFreeCall("B", line_tracker_b1, line_tracker_b2);    
                    }                  
                   clearInterval(ajax_interval);
                }else{
                    if(res.running == "null"){
                        clearInterval(ajax_interval);
                    }
                }
            },
            error:function(res){
                clearInterval(ajax_interval);
            }
        });
    },500)

}

function line_tracker_lab_btn_update_cb() {
    data = { "l_min": line_tracker_l1, "l_max": line_tracker_l2, "a_min": line_tracker_a1, 
                 "a_max": line_tracker_a2, "b_min": line_tracker_b1, "b_max": line_tracker_b2 };
    var xmlhttp = new XMLHttpRequest();
    xmlhttp.open("POST", "/data_to_device", true);
    xmlhttp.setRequestHeader("Content-Type", "application/json");
    xmlhttp.send(JSON.stringify(data));

   

}

function line_tracker_lab_btn_mode_cb() {
    var xmlhttp = new XMLHttpRequest();
    xmlhttp.open("POST", "/data_to_device", true);
    xmlhttp.setRequestHeader("Content-Type", "application/json");
    if (line_tracker_btn_mode_flag == 0) {
        $("#line_tracker_lab_mode").html("change to RGB Mode");
        $("#line_tracker_lab_mode").css({
            background: '#ef5773'
        });
        line_tracker_btn_mode_flag = 1;
        xmlhttp.send(JSON.stringify({ "mode": "mask" }));
    } else {
        $("#line_tracker_lab_mode").html("change to Mask Mode");
        $("#line_tracker_lab_mode").css({
            background: '#222831'
        });
        line_tracker_btn_mode_flag = 0;
        xmlhttp.send(JSON.stringify({ "mode": "rgb" }));
    }

}

//color_tracker
function color_tracker_lab_roi_update() {
    draw_rect_flag = 0;
    $.ajax({
        type: "post",
        url: "/color_lab",
        contentType: "application/json;charset=UTF-8",
        data: roi_payload,
        dataType:"json",
        success: function (res) {
            if(res  ==ok){

            }else{
               
            }
        },
        error: function (res) {
        }
    })
    ajax_interval = setInterval(function(){
        $.ajax({
            type:"post",
            url:"/data_from_device",
            dataType:"json",
            success:function(res){
                if(res != null && res.running == "Color Tracker"){
                    if(res.l_min !=null){
                        color_tracker_l1 = res.l_min;
                        color_tracker_l2 = res.l_max;
                        color_tracker_a1 = res.a_min;
                        color_tracker_a2 = res.a_max;
                        color_tracker_b1 = res.b_min;
                        color_tracker_b2 = res.b_max;
                        color_tracker_sidlerBarFreeCall("L", color_tracker_l1, color_tracker_l2);
                        color_tracker_sidlerBarFreeCall("A", color_tracker_a1, color_tracker_a2);   
                        color_tracker_sidlerBarFreeCall("B", color_tracker_b1, color_tracker_b2);
                    }                      
                   clearInterval(ajax_interval);
                }else{
                    if(res.running == "null"){
                        clearInterval(ajax_interval);
                    }
                }
            },
            error:function(res){
                clearInterval(ajax_interval);
            }
        });
    },500)

}

function color_tracker_lab_btn_update_cb() {
    data = { "l_min": color_tracker_l1, "l_max": color_tracker_l2, "a_min": color_tracker_a1, "a_max": color_tracker_a2, "b_min": color_tracker_b1, "b_max": color_tracker_b2 };
    var xmlhttp = new XMLHttpRequest();
    xmlhttp.open("POST", "/data_to_device", true);
    xmlhttp.setRequestHeader("Content-Type", "application/json");
    xmlhttp.send(JSON.stringify(data));

}

function color_tracker_lab_btn_mode_cb() {
    var xmlhttp = new XMLHttpRequest();
    xmlhttp.open("POST", "/data_to_device", true);
    xmlhttp.setRequestHeader("Content-Type", "application/json");

    if (color_tracker_btn_mode_flag == 0) {
        $("#color_tracker_lab_mode").html("change to RGB Mode");
        $("#color_tracker_lab_mode").css({
            background: '#ef5773'
        });
        color_tracker_btn_mode_flag = 1;
        xmlhttp.send(JSON.stringify({ "mode": "mask" }));
    } else {
        $("#color_tracker_lab_mode").html("change to Mask Mode");
        $("#color_tracker_lab_mode").css({
            background: '#222831'
        });
        color_tracker_btn_mode_flag = 0;
        xmlhttp.send(JSON.stringify({ "mode": "rgb" }));
    }
}

//shape_detector
function shape_detector_btn_demarcate_cb(){
    var xmlhttp = new XMLHttpRequest();
    xmlhttp.open("POST", "/data_to_device", true);
    xmlhttp.setRequestHeader("Content-Type", "application/json");
    xmlhttp.send(JSON.stringify({ "operation": "update" }));
    draw_rect_flag = 0;
}

//shape_matching
function shape_matching_btn_matching_cb(){
    clearDynamicFuncArea();
    loadStream();
    xmlhttp.open("POST", "/data_to_device", true);
    xmlhttp.setRequestHeader("Content-Type", "application/json");
    // xmlhttp.send(JSON.stringify({ "operation": "matching" }));
}
function shape_matching_btn_demarcate_cb(){
    var xmlhttp = new XMLHttpRequest();
    xmlhttp.open("POST", "/data_to_device", true);
    xmlhttp.setRequestHeader("Content-Type", "application/json");
    xmlhttp.send(JSON.stringify({"config" : "web_update_data"
    ,"operation": "update"  }));
}
function shape_matching_btn_reset_cb(){
    clearDynamicFuncArea();
    loadStream();
    shape_matching_file_reset();
    shape_matching_btn_add_cb();
    var xmlhttp = new XMLHttpRequest();
    xmlhttp.open("POST", "/data_to_device", true);
    xmlhttp.setRequestHeader("Content-Type", "application/json");
    xmlhttp.send(JSON.stringify({"config" : "web_update_data"
    ,"operation": "reset" }));
}


//object_recognition
function yolo_btn_run_cb(){
    func_id = '1';
    func_name = 'object_recognition';
    
    createLoading();
    $.ajax({
        type: "POST",
        url: "/func",
        contentType: "application/json;charset=UTF-8",
        data: JSON.stringify({ "type_id": func_id, "type_name": func_name,"args": [yolo_args]}),
        success: function (res) {
            destroyLoading()
        },
        error: function (res) {
            destroy_loading = setTimeout("destroyLoading()", 500);
        }
    })
}

function yolo_btn_reset_cb(){
    // var xmlhttp = new XMLHttpRequest();
    // xmlhttp.open("POST", "/data_to_device", true);
    // xmlhttp.setRequestHeader("Content-Type", "application/json");
    // xmlhttp.send(JSON.stringify({ "operation": "reset" }));
}

function yolo_model_remove(ts){
    if(yolo_args == "null"){
        yolo_model_input_remove(ts);
        return;
    }

    if(!confirm("confirm delete model  !")){
        return;
    }
   
    $.ajax({
        type:"post",
        url:"/upload/delmodels",
        contentType: "application/json;charset=UTF-8",
        data: JSON.stringify({"delmodel":yolo_args}),
        success:function(res){
            if(res == "ok"){
                yolo_model_input_remove(ts);
                alert("delete model success  !")
            }else{
                try{
                   let r =  JSON.parse(res);
                   alert(r.error)
                }catch(error){
                    alert("delete model failed  !")
                }
            }
        },
        error:function(res){
            alert("delete model failed  !")
        }
    })
}


/* 切换*/
function switch_sys_mode(){
    if(confirm("Confirm to enter notebook mode? Current page will be closed.")){
        $.ajax({
            type:"post",
            url:"/switch_sys_mode",
            success:function(res){
            }
        })

        alert("Please refresh this page later");
    }
}

