

var gallery_section_flag = 0;
function gallery_init(){
    $(".photograph").hide();
    $(".gallery-img-card-button-group").hide()
    $("#gallery").click(function(event){
        if(gallery_section_flag ==0){
            $(".gallery-section").css({
                "bottom":"0px",
                "transition" : "bottom .25s linear"
            });
            gallery_section_flag= 1;
         }else{
            $(".gallery-section").css({
                "bottom":"-400px",
                "transition" : "bottom .25s linear"
            });
            gallery_section_flag= 0;
         }
         event.stopPropagation();    
    })

    $(".gallery-section-close").click(function(){
        $("#gallery").click();
    })

    document.querySelector(".container").addEventListener('click',function(e){
        if(gallery_section_flag==1){
            $("#gallery").click();
        }
    },false)
}

//保存文件
function downLoadImage(ts){
    var a = document.createElement("a");
    var url =  $(ts).parent().parent().find("img").attr('src');
    var event = new MouseEvent('click');
    a.download = 'unitV2';
    a.href = url;
    a.dispatchEvent(event);
}
//删除图片
function deleteImage(dom){
    $(dom).parent().parent().remove()
}