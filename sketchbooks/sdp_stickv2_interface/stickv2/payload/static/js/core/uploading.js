
function uploading_init(){
    $("#uploading-menu-close").click(function(){
        uploading_hide();
    })
}

function uploading_hide(){
    $(".uploading").fadeOut('fast');
    $(".uploading-success").hide();
    $(".uploading-failed").hide();
    $(".uploading-res").hide();
    $(".uploading-text").html("uploading")
    $(".uploading-res").html("")
    setTimeout(function(){
        $(".uploading-process >div").css({
            "width": "0"
        })
    },10)
}

function uploading_success(){
   setTimeout(function(){
    $(".uploading-success").show();
    $(".uploading-text").html("upload successed")
   },2000)
}

function uploading_failure(res){
    setTimeout(function(){
     $(".uploading-failed").show();
     $(".uploading-res").show();
     $(".uploading-text").html("upload failed")
     $(".uploading-res").html(res)
    },2000)
 }