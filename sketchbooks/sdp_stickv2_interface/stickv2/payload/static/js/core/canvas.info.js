var canvas_info_flag = 0;
function canvasInfoInit(){
    let canvas_func_title_height = $(".canvas-func-titel").outerHeight()-1;
    $(".canvas-func-tips svg").on('click',function(){
        if(canvas_info_flag == 0){
            $(".canvas-func-info").css({
                "display":"block",
                "top":canvas_func_title_height
            })
            $("#canvas-func-tips-svg").css({
                "transform": "rotate(180deg)"
            })
             canvas_info_flag = 1
        }else{
            canvas_info_flag = 0
           
            $(".canvas-func-info").css({
                "display":"none"
            })
            $("#canvas-func-tips-svg").css({
                "transform": "rotate(0deg)"
            })
        }   
    })
}