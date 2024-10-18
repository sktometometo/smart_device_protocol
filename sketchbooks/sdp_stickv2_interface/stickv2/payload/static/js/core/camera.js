function camera_init(cvs){
    $("#camera").off('click')
    $("#camera").on('click', function (event) {
        if(gallery_section_flag ==0){
            $("#gallery").click();
        }
        photograph(cvs);
        event.stopPropagation();
    })
}
//拍照
function photograph(cvs) {
    let card_div = $('<div></div>');
    let data_src = cvs.toDataURL();
    card_div.addClass('gallery-img-card');
    card_div.append('<img src=' + data_src + '>');
    card_div.append(`<div class="gallery-img-card-button-group">
                    <button onclick="downLoadImage(this)">
                    <svg t="1616399718057" class="icon" viewBox="0 0 1024 1024" version="1.1" xmlns="http://www.w3.org/2000/svg" p-id="15195" width="200" height="200"><path d="M721.12 540.16 540.128 721.28C532.448 728.96 521.952 731.2 512 729.28 502.048 731.2 491.552 728.96 483.872 721.28L302.88 540.16C290.368 527.68 290.368 507.52 302.88 494.72 315.36 482.24 335.616 482.24 348.128 494.72L480 626.88 480 288C480 270.4 494.336 256 512 256 529.696 256 544 270.4 544 288L544 626.88 675.872 494.72C688.384 482.24 708.672 482.24 721.12 494.72 733.632 507.52 733.632 527.68 721.12 540.16L721.12 540.16ZM512 0C229.216 0 0 229.12 0 512 0 794.88 229.216 1024 512 1024 794.784 1024 1024 794.88 1024 512 1024 229.12 794.784 0 512 0L512 0Z" p-id="15196"></path>
                    </svg>
                    </button>
                    <button onclick="deleteImage(this)"><svg t="1615280991766" class="icon" viewBox="0 0 1024 1024" version="1.1"
                            xmlns="http://www.w3.org/2000/svg" p-id="12480" style="width: 21px;height: 21px;">
                            <path style="fill: red;"
                                d="M512 0C229.29408 0 0 229.248 0 512c0 282.76736 229.26336 512 512 512 282.76736 0 512-229.18144 512-512C1024 229.29408 794.76736 0 512 0z m209.90976 654.09024a47.5904 47.5904 0 0 1 14.08512 33.90976 47.95392 47.95392 0 0 1-47.99488 47.99488 47.75936 47.75936 0 0 1-33.90976-14.08512L512 579.89632l-141.99296 142.05952a47.9488 47.9488 0 0 1-34.00704 14.08512c-26.45504 0-47.95904-21.45792-47.95904-48.01536A47.68768 47.68768 0 0 1 302.08 654.15168L444.15488 512 302.1056 369.95584a47.88224 47.88224 0 0 1-14.03904-33.95584 47.99488 47.99488 0 0 1 81.92-33.95584l141.99296 142.07488L654.07488 302.08a47.92832 47.92832 0 0 1 33.90976-14.03904c26.48576 0 47.99488 21.45792 47.99488 47.99488a47.75424 47.75424 0 0 1-14.08512 33.95584l-141.99808 141.99296 142.01344 142.1056z m0 0"
                                fill="#272636" p-id="12481"></path>
                        </svg></button>
                    </div>`)
    $(".gallery-main-section").append(card_div);
    $(card_div).find('.gallery-img-card-button-group').hide()
   
    $(".gallery-img-card img").click(function(event){
        $(".gallery-img-card-button-group").hide()
        $(".gallery-img-card img").css({
            "box-shadow":"none"
        })
         
        $(this).next().fadeIn('fast');         
        $(this).css({
           "box-shadow":"0px 0px 1px 5px #fff"
        })
    })
}


