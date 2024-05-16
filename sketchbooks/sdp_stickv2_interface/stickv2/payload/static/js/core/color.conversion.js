function LABToRGBHEX(L, A, B) {
    return rgbToHEX(XYZToRGB(LABToXYZ(L255To100(L), A, B)));
}
//L量化到0~100的范围
function L255To100(l) {
    return l * 100.0 / 255.0;
}
// lab to xyz
function LABToXYZ(l, a, b) {
    let fx, fy, fz;
    let rx, ry, rz;
    fy = (l + 16.0) / 116.0;
    fx = (a - 128) / 500.0 + fy;
    fz = fy - (b - 128) / 200.0;
    if (Math.pow(fy, 3.0) > 0.008856) {
        ry = Math.pow(fy, 3.0);
    } else {
        ry = (fy - 16.0 / 116.0) / 7.787;
    }
    if (Math.pow(fx, 3.0) > 0.008856) {
        rx = Math.pow(fx, 3.0);
    } else {
        rx = (fx - 16.0 / 116.0) / 7.787;
    }
    if (Math.pow(fz, 3.0) > 0.008856) {
        rz = Math.pow(fz, 3.0);
    } else {
        rz = (fz - 16.0 / 116.0) / 7.787;
    }
    return {
        rx: rx *= 0.950456,
        ry: ry *= 1.0,
        rz: rz *= 1.088754
    }
}
//xyz to rgb 
function XYZToRGB(obj) {
    let RR, GG, BB;
    RR = 3.2404542 * obj.rx - 1.5371385 * obj.ry - 0.4985314 * obj.rz;
    GG = -0.9692660 * obj.rx + 1.8760108 * obj.ry + 0.0415560 * obj.rz;
    BB = 0.0556434 * obj.rx - 0.2040259 * obj.ry + 1.0572252 * obj.rz;
    RR = gammaXYZToRGB(RR);
    GG = gammaXYZToRGB(GG);
    BB = gammaXYZToRGB(BB);
    RR = clip255((RR * 255 + 0.5)) >>> 0;
    GG = clip255((GG * 255 + 0.5)) >>> 0;
    BB = clip255((BB * 255 + 0.5)) >>> 0;
    return {
        r: RR,
        g: GG,
        b: BB
    }
}
function clip255(param) {
    if (param < 0) {
        return 0;
    } else if (param > 255) {
        return 255;
    } else {
        return param;
    }
}
function gammaXYZToRGB(x) {
    return x > 0.0031308 ? (1.055 * Math.pow(x, (1 / 2.4)) - 0.055) : (x * 12.92);
}
function rgbToHEX(obj) {
    let hex = "#" + ((1 << 24) + (obj.r << 16) + (obj.g << 8) + obj.b).toString(16).slice(1);
    return hex;
}


// rgb  to xyz
function RGBToXYZ(obj) {
    let RR = gammaXYZToLAB(obj.r / 255.0)
    let GG = gammaXYZToLAB(obj.g / 255.0)
    let BB = gammaXYZToLAB(obj.b / 255.0);
    return {
        x: 0.4124564 * RR + 0.3575761 * GG + 0.1804375 * BB,
        y: 0.2126729 * RR + 0.7151522 * GG + 0.0721750 * BB,
        z: 0.0193339 * RR + 0.1191920 * GG + 0.9503041 * BB
    }
}


// xyz to lab
function XYZToLAB(obj) {
    let fx, fy, fz;
    let rx, ry, rz;
    rx = obj.x / 0.950456;
    ry = obj.y / 1.0;
    rz = obj.z / 1.088754;
    if (ry > 0.008856) {
        fy = Math.pow(ry, 1.0 / 3.0);
    } else {
        fy = 7.787 * ry + 16.0 / 116.0
    }
    if (rx > 0.008856) {
        fx = Math.pow(rx, 1.0 / 3.0);
    } else {
        fx = 7.787 * rx + 16.0 * 116.0;
    }
    if (rz > 0.008856) {
        fz = Math.pow(rz, 1.0 / 3.0)
    } else {
        fz = 7.787 * rz + 16.0 * 116.0;
    }
    return {
        l: 116.0 * fy - 16.0 > 0.0 ? 116.0 * fy - 16.0 : 0.0,
        a: 500.0 * (fx - fy),
        b: 200.0 * (fy - fz)
    }
}

function gammaXYZToLAB(x) {
    return x > 0.04048 ? Math.pow((x + 0.055) / 1.055, 2.4) : (x / 12.92);
}

// function formatRGB(val) {
//     const hex_exp = /(^#[0-9A-F]{6}$)|(^#[0-9A-F]{3}$)/i;
//     const rgb_exp = /^rgb/;
//     const hsl_exp = /^hsl/;
//     if (rgb_exp.test(val)) {
//         r = rgb.exec(val);
//     } else if (hex_exp.test(val)) {
//         let hex = val.replace(hex_exp,(m,r,g,b)=> r+r+g+g+b+b)
//     } else if (hsl_exp.test(val)) {
//     } else {
//         return -1;
//     }
//     return {
//         r: r,
//         g: g,
//         b: b
//     }
// }

