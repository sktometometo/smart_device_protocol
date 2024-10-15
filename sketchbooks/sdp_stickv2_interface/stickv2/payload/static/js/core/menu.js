function menu_init(){
    $("#menu-btn").on('click',function(){
        if(render_string_flag == 0){
            $("#menu-btn-label").css({
                "fill":"#00adb5"
            })
            $("#menu-btn-label-circle").css(
                {'d':'path("M500 278.528a247.125333 247.125333 0 1 0 245.76 245.76 247.125333 247.125333 0 0 0-245.76-245.76z")'})
            render_string_flag =1
        }else{
            $("#menu-btn-label").css({
                "fill":"#000"
            })

            $("#menu-btn-label-circle").css(
                {'d':'path("M1104.554667 278.528a247.125333 247.125333 0 1 0 245.76 245.76 247.125333 247.125333 0 0 0-245.76-245.76z")'})
            render_string_flag =0
        }
    })
}

function menuButtonActive(ts){
    if($('.funcIcon-active').length>0){
        $('.funcIcon-active').attr('style',"fill:#b9baba");
        $('.funcIcon-active').removeClass("funcIcon-active");
    }       
    $('.under-line').remove();
    $(ts).find('svg').addClass('funcIcon-active');
    $(ts).find('svg').attr('style',"fill:#fff");
    $(ts).last().append("<div class='under-line'></div> ");
}