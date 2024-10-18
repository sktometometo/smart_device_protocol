
var render_string_flag = 0;
//请求 渲染视频流的数据
function requestStreamData() {
    fetch('/render_items',{method:"POST"}).then((response)=>{
        fetchStream(response.body)
    }).catch(error=>{
        console.log("requestStreamData error " +error)
    })
}

function fetchStream(stream) {
    const reader = stream.getReader(); 
    // read() 返回了一个promise
    // 当数据被接收时resolve
    reader.read().then(function processText({ done, value }) {
      // Result对象包含了两个属性:
      // done  - 当stream传完所有数据时则变成true
      // value - 数据片段。当done不为true时永远为undefined
      let str = String.fromCharCode.apply(null, value);
      if(str!=null && str.length>0){     
            try {
                stream_res= JSON.parse(str.split('|')[0]);    
            } catch (error) {

            }
        } else{
            res = null;
        }
      if (done) {
        console.log("Stream complete");
        return;
      }
      // 再次调用这个函数以读取更多数据
      return reader.read().then(processText);
    });
  }

function renderStream(stream_type) {
    stream_thickness = 7;
    if (stream_type == 'string' && render_string_flag == 0) {
        renderString();
    } else if (stream_type == "rectangle") {
        renderRectangle();
    } else if (stream_type == "circle") {
        renderCircle();
    } else if (stream_type == "point") {
        renderPoint();
    } else if (stream_type == "line") {
        renderLine();
    }
    else if (stream_type == "polygon") {
        renderPolygon();
    }
    ctx.setLineDash([]);
}
function renderRectangle() {
    ctx.strokeStyle = stream_color;
    ctx.lineWidth = stream_thickness; 
    ctx.beginPath();
    ctx.moveTo(stream_x1, stream_y1);
    ctx.lineTo(stream_x1 + stream_w, stream_y1);
    ctx.lineTo(stream_x1 + stream_w, stream_y1 + stream_h);
    ctx.lineTo(stream_x1, stream_y1 + stream_h);
    ctx.closePath();
    ctx.stroke();
}
function renderCircle() {
    ctx.lineWidth = stream_thickness;
    ctx.strokeStyle = stream_color;
    ctx.beginPath();
    // 绘制圆的路径**
    ctx.arc(stream_x1, stream_y1, stream_r, 0, Math.PI * 2, false);
    ctx.stroke();
}
function renderPoint() {
    ctx.strokeStyle = stream_color;
    ctx.lineWidth = stream_thickness;
    ctx.lineCap = "round";
    ctx.beginPath();
    ctx.moveTo(stream_x1, stream_y1);
    ctx.lineTo(stream_x1, stream_y1+10);
    ctx.stroke();
    ctx.moveTo(stream_x1-5,stream_y1+5);
    ctx.lineTo(stream_x1+5, stream_y1+5);
    ctx.stroke();
}
function renderString() {

    // stream_color = "#123123"
    // stream_data = "test反色"
    // stream_x1 = 150
    // stream_y1 = 150
    stream_thickness = 1.5;

    ctx.fillStyle = stream_color;
    let str_width = ctx.measureText(stream_data).width;
    ctx.fillRect(stream_x1, stream_y1-20 ,str_width,16*1.1); 
    ctx.fillStyle =  "#FFFFFF"
    ctx.font = '1em Roboto';
    ctx.textBaseline = 'top';
    ctx.fillText(stream_data, stream_x1, stream_y1-16);

}

function renderLine() {
    ctx.strokeStyle = stream_color
    ctx.lineWidth = stream_thickness;
    ctx.beginPath();
    ctx.moveTo(stream_x1, stream_y1);
    ctx.lineTo(stream_x2, stream_y2);
    ctx.closePath();
    ctx.stroke();
}

function renderPolygon(){
    ctx.strokeStyle = stream_color
    ctx.lineWidth = stream_thickness;
    if(stream_array_x != null &&  stream_array_y != null){
        ctx.beginPath();
        ctx.moveTo(stream_array_x[0], stream_array_y[0]);
        for(let i = 0,len = stream_array_x.length;i<len;i++){
            if(i == len-1){
                ctx.lineTo(stream_array_x[0], stream_array_y[0]);
            }else{
           
            ctx.lineTo(stream_array_x[i+1], stream_array_y[i+1]);
            }
        }
        ctx.closePath();  
        ctx.stroke();
    }
}
function clearRender(){
    stream_type = null;
    stream_x1= null; 
    stream_y1= null; 
    stream_x2= null; 
    stream_y2= null; 
    stream_w= null; 
    stream_h= null; 
    stream_r= null; 
    stream_color= null;
    stream_data= null; 
    stream_thickness= null;
    stream_array_x = null;
    stream_array_y = null;
}

//字体反色
function oppositeColor(a){
    a=a.replace('#','');
    var c16,c10,max16=15,b=[];
    for(var i=0;i<a.length;i++){   
        c16=parseInt(a.charAt(i),16);
        c10=parseInt(max16-c16,10);
        b.push(c10.toString(16));  
    }
    return '#'+b.join('');
}

function ColorReverse(OldColorValue){
    var OldColorValue="0x"+OldColorValue.replace(/#/g,"");
     var str="000000"+(0xFFFFFF-OldColorValue).toString(16);
     return "#" + str.substring(str.length-6,str.length);
}
