//canvas loading
var destroy_loading,show_loading_menu;
document.onreadystatechange = initloading;
function initloading() {
    if (document.readyState == 'complete') {
        $(".loading").css({
            "display": "none"
        });
        $('button').removeAttr("disabled")
    } else {
        $('button').attr("disabled", "disabled")
    }
}

function createLoading(){
    $(".loading-menu").css({
        "display": "none"
    });
     $(".loading").css({
         "display": "flex"
     });
    $('button').attr("disabled", "disabled")
    // show_loading_menu= setTimeout("showLoadingMenu()",3000);
    $(".loading-menu svg").on("click",function(){
        destroyLoading();
    })
}

function destroyLoading(){
    $(".loading").css({
        "display": "none"
    });       
    $('button').removeAttr("disabled");
    clearTimeout(destroy_loading)
    clearTimeout(show_loading_menu)
}

function showLoadingMenu(){
    $(".loading-menu").css({
        "display": "flex"
    });
}

