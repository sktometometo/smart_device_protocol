//file upload
var firmware_file = $("#firmware_file");
var model_file = $("#model_file");
var package_suffix = { 'zip': "zip", 'tar': "tar" ,'gz':'gz'};

function file_upload_init() {
    $(".firmware .progress").hide();
    $(".firmware-submit").hide();

    firmware_file.change(function(){
        let path = firmware_file.val();
        let suffix = path.substring(path.lastIndexOf('.') + 1, path.length).toLowerCase();
        if (!(suffix in package_suffix)) {        
            $("#firmware_file_label span").html("UPLOAD FILE");
            alert("Support zip, tar, gz format!")
            return;
        }
        
        $("#firmware_file_label span").html($("#firmware_file")[0].files[0].name);
        firmware_file_upload_init()
    })

    //固件
    $("#firmware_file_upload").click(function () {
        var formData = new FormData($("#firmware_file_form")[0]);     
        if(!confirm("Warning,The firmware will be updated after uploading!")){
            return;
        }
        $.ajax({           
            type: 'POST',
            url: '/upload/firmware',          
            data: formData,
            processData:false,
            contentType:false,
            xhr:function(){
                var xhr = $.ajaxSettings.xhr();
                if(xhr.upload){
                    xhr.upload.addEventListener('progress',function(e){
                        var loaded =e.loaded;
                        var total = e.total;
                        var percent = Math.floor(100*loaded/total)+"%";
                        $(".firmware .progress div").css({
                            "width": percent
                        })
                    })
                }
                return xhr;
            },
            success:function(res){
                if(res == "ok"){
                    $(".firmware .progress").fadeOut('fast');
                }else{
                    firmware_file_upload_failed()
                }
            },
            error:function(res){
                firmware_file_upload_failed()
            },
        })
    })
  
}

function  firmware_file_upload_init(){
    $(".firmware .progress").fadeIn('fast');
    $(".firmware-submit").fadeIn('fast');
    $(".firmware .progress div").css({
        "width":"0"
    })
}
function firmware_file_upload_successed(){
    setTimeout(function(){

    },2000)
}
function firmware_file_upload_failed(){
    setTimeout(function(){
        $("#firmware_file_label span").html("UPLOAD FILE");
        $(".firmware .progress").fadeOut('fast');
        $(".firmware-submit").hide();
    },2000)
}