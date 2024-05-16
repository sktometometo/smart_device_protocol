
var mouse_flag = 0;
var stop_draw_flag
var stop_load_flag;
var draw_rect_flag = 0;
//画框坐标
var x1 = 0, y1 = 0, x2 = 0, y2 = 0;
/*loadStream*/
function loadStream() {
    stop_load_flag = setInterval(unitV2Stream, 20);
}
function unitV2Stream() {
    try {
        ctx.drawImage(img, 0, 0, 640, 480);
    } catch (error) {
        stopLoadStream()
    }
    if(stream_res != null){
        if(stream_res.render == 0){
            clearRender();
        }else{
            for (let i = 0; i < stream_res.items.length; i++) {
                stream_type = stream_res.items[i].type;
                stream_array_x = stream_res.items[i].x;
                stream_array_y = stream_res.items[i].y;
                stream_x1 = stream_res.items[i].x1;
                stream_y1 = stream_res.items[i].y1;
                stream_x2 = stream_res.items[i].x2;
                stream_y2 = stream_res.items[i].y2;
                stream_w = stream_res.items[i].w1;
                stream_h = stream_res.items[i].h1;
                stream_r = stream_res.items[i].r1;
                stream_color = stream_res.items[i].color;
                stream_thickness = stream_res.items[i].thickness
                stream_data = stream_res.items[i].payload;
                renderStream(stream_type)
            }
        }
    }
}
/*画框*/
function draw() {
    unitV2Stream();
    if(draw_rect_flag == 1) {
        ctx.beginPath();
        ctx.moveTo(x1, y1);
        ctx.lineTo(x2, y1);
        ctx.lineTo(x2, y2);
        ctx.lineTo(x1, y2);
        ctx.closePath();
        ctx.stroke();
    }
}

function drawRect() {
    ctx.strokeStyle = '#00cc11'
    ctx.lineWidth = 5;
    stop_draw_flag = setInterval(draw, 20);
    canvas.onmousedown = function (e) {
        let startX, startY;
        startX = e.offsetX, startY = e.offsetY;
        if (mouse_flag == 0) {
            x1 = startX;
            y1 = startY;
            x2 = x1;
            y2 = y1;
            draw_rect_flag = 1;
        }
        mouse_flag = 1
        canvas.onmousemove = function (e) {
            if (mouse_flag == 1) {
                x2 = e.pageX - this.offsetLeft;
                y2 = e.pageY - this.offsetTop;
            }
        }
        canvas.onmouseup = function (e) {
            mouse_flag = 0
            data = { "x": parseInt(x2 - x1 > 0 ? startX : startX - Math.abs(x2 - x1)), "y": parseInt(y2 - y1 > 0 ? startY : startY - Math.abs(y2 - y1)), "w": Math.abs(x1 - x2), "h": Math.abs(y1 - y2) };         
            roi_payload = JSON.stringify(data);
            if (func_id == 4) {
                color_tracker_lab_roi_update();
            }
            else if(func_id == 5){
                line_tracker_lab_roi_update();
            }
            else if (func_id == 6) {
                target_tracker_btn_update_cb();
            }
        }
    }
}

function clearRect() {
    x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    ctx.drawImage(img, 0, 0, 640, 480);
}
/*画框结束*/


//注销画框事件
function offCanvasEvent() {
    canvas.onmousedown = null;
    canvas.onmousemove = null;
    canvas.onmouseup = null;
}

function stopLoadStream(){
    clearInterval(stop_load_flag);
    clearInterval(stop_draw_flag);
}