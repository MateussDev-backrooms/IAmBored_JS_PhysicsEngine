//vars
let canvas
let ctx
let substep_cnt = 8

let gravity = {x:0, y:1000}
let mouse = {x: 0, y: 0}

let display_debug = true

class SoftBody{
    constructor(ofsx, ofsy, w, h, elemx, elemy, softness, color) {
        this.offset_x = ofsx;
        this.offset_y = ofsy;
        this.w = w;
        this.h = h;
        this.elx = elemx;
        this.ely = elemy;
        this.mesh = []
        this.mesh_vel = []
        this.mesh_prev = []
        for(let mx = 0; mx < this.elx; mx++) {
            for(let my = 0; my < this.ely; my++) {
                this.mesh.push({x: mx*(this.w/this.elx)+ofsx, y: my*(this.h/this.ely)+ofsy})
                this.mesh_vel.push({x: 0, y: 0})
                this.mesh_prev.push({x: mx*(this.w/this.elx)+ofsx, y: my*(this.h/this.ely)+ofsy})
            }
        }
        this.prev = Date.now()
        this.softness = softness
        this.color = color
    }
    init() {

    }
    update() {
        //set pos to be previous before we change it

        //update delta time
        const slowdown = 1 //serves as the multiplier that delta time will be divided at. The greated the number - the slower the time passes
        const delta_time = (Date.now()-this.prev)/(1000*(slowdown*substep_cnt))
        this.prev = Date.now()
        // console.log(delta_time)

        //solve 
        for(let sub=0; sub<substep_cnt; sub++) {
            //step A - apply transformation
            for(let x = 0; x < this.elx; x++) {
                for(let y = 0; y < this.ely; y++) {
                    //calculate gravity
                    this.mesh_vel[x*this.elx + y] = v_add(this.mesh_vel[x*this.elx + y], v_mul_n(gravity, delta_time))

                    //save previous pos
                    this.mesh_prev[x*this.elx + y] = {x: this.mesh[x*this.elx + y].x, y:this.mesh[x*this.elx + y].y}
    
                    //add saved velocity to current point
                    this.mesh[x*this.elx + y] = v_add(this.mesh[x*this.elx + y], v_mul_n(this.mesh_vel[x*this.elx + y], delta_time))
    
                }
            }
            //step B - apply constraints

            

            //distance
            for(let x = 0; x < this.elx; x++) {
                for(let y = 0; y < this.ely; y++) {
                    //solve for horizontal neighbor
                    if(x + 1 < this.elx) {
                        //get elems
                        let x1 = this.mesh[x*this.elx + y]
                        let x2 = this.mesh[(x+1)*this.elx + y]
                        
                        //calculate distances
                        const restl = this.w / this.elx

                        //delta calculations
                        let ddist = v_length(v_sub(x1, x2)) - restl
                        let dpos = v_sub(x2, x1)
                        dpos = v_mul_n(dpos, 1 / v_length(dpos))

                        let delta1 = v_mul_n(v_mul_n(dpos, ddist*this.softness), 1)
                        let delta2 = v_mul_n(v_mul_n(dpos, -ddist*this.softness), 1)

                        //apply delta offsets
                        this.mesh[x*this.elx + y] = v_add(this.mesh[x*this.elx + y], delta1)
                        this.mesh[(x+1)*this.elx + y] = v_add(this.mesh[(x+1)*this.elx + y], delta2)
                    }
                    //solve for vertical neighbor
                    if(y + 1 < this.ely) {
                        //get elems
                        let x1 = this.mesh[x*this.elx + y]
                        let x2 = this.mesh[x*this.elx + y+1]

                        //calculate distances
                        const restl = this.h / this.ely

                        //delta calculations
                        let ddist = v_length(v_sub(x1, x2)) - restl
                        let dpos = v_sub(x2, x1)
                        dpos = v_mul_n(dpos, 1/v_length(dpos))

                        let delta1 = v_mul_n(v_mul_n(dpos, ddist*this.softness), 1)
                        let delta2 = v_mul_n(v_mul_n(dpos, -ddist*this.softness), 1)

                        //apply delta offsets
                        this.mesh[x*this.elx + y] = v_add(this.mesh[x*this.elx + y], delta1)
                        this.mesh[x*this.elx + y+1] = v_add(this.mesh[x*this.elx + y+1], delta2)
                    }
                    //solve for diagonal neighbor L to R
                    if(x + 1 < this.elx && y + 1 < this.ely) {
                        //get elems
                        let x1 = this.mesh[x*this.elx + y]
                        let x2 = this.mesh[(x+1)*this.elx + y+1]

                        //calculate distances
                        const restl = this.w / this.elx

                        //delta calculations
                        let ddist = v_length(v_sub(x1, x2)) - Math.SQRT2*restl
                        let dpos = v_sub(x2, x1)
                        dpos = v_mul_n(dpos, 1/v_length(dpos))

                        let delta1 = v_mul_n(v_mul_n(dpos, ddist*this.softness), 1)
                        let delta2 = v_mul_n(v_mul_n(dpos, -ddist*this.softness), 1)

                        //apply delta offsets
                        this.mesh[x*this.elx + y] = v_add(this.mesh[x*this.elx + y], delta1)
                        this.mesh[(x+1)*this.elx + y+1] = v_add(this.mesh[(x+1)*this.elx + y+1], delta2)
                    }
                    //solve for diagonal neighbor R to L
                    if(x + 1 < this.elx && y + 1 < this.ely) {
                        //get elems
                        let x1 = this.mesh[(x+1)*this.elx + y]
                        let x2 = this.mesh[x*this.elx + y+1]

                        //calculate distances
                        const restl = this.w / this.elx

                        //delta calculations
                        let ddist = v_length(v_sub(x1, x2)) - Math.SQRT2*restl
                        let dpos = v_sub(x2, x1)
                        dpos = v_mul_n(dpos, 1/v_length(dpos))

                        let delta1 = v_mul_n(v_mul_n(dpos, ddist*this.softness), 1)
                        let delta2 = v_mul_n(v_mul_n(dpos, -ddist*this.softness), 1)
                        // console.log(delta1)
                        // console.log(this.softness)

                        //apply delta offsets
                        this.mesh[(x+1)*this.elx + y] = v_add(this.mesh[(x+1)*this.elx + y], delta1)
                        this.mesh[x*this.elx + y+1] = v_add(this.mesh[x*this.elx + y+1], delta2)
                    }
                }
            }

            //bounds
            for(let x = 0; x < this.elx; x++) {
                for(let y = 0; y < this.ely; y++) {
                    if(this.mesh[x*this.elx + y].y > canvas.height) {
                        this.mesh[x*this.elx + y].y = canvas.height
                        this.mesh_vel[x*this.elx + y].y = -this.mesh_vel[x*this.elx + y].y
                    }
                    if(this.mesh[x*this.elx + y].y < 0) {
                        this.mesh[x*this.elx + y].y = 0
                        this.mesh_vel[x*this.elx + y].y = -this.mesh_vel[x*this.elx + y].y
                    }
                    if(this.mesh[x*this.elx + y].x > canvas.width) {
                        this.mesh[x*this.elx + y].x = canvas.width
                    }
                    if(this.mesh[x*this.elx + y].x < 0) {
                        this.mesh[x*this.elx + y].x = 0
                    }
                }
            }

            //step C - XPBD formula
            for(let i in this.mesh) {
                this.mesh_vel[i] = v_mul_n(v_add(this.mesh[i], v_mul_n(this.mesh_prev[i], -1)), 1 / delta_time)
                
            }
        }
    }
    render() {
        for(let x = 0; x < this.elx-1; x++) {
            for(let y = 0; y < this.ely-1; y++) {
                
                if(display_debug) {
                    ctx.strokeStyle = this.color
                    v_drawLine(this.mesh[x*this.elx + y], this.mesh[x*this.elx + y+1], 4)
                    v_drawLine(this.mesh[x*this.elx + y], this.mesh[(x+1)*this.elx + y], 4)
                    v_drawLine(this.mesh[(x+1)*this.elx + y], this.mesh[(x+1)*this.elx + y+1], 4)
                    v_drawLine(this.mesh[x*this.elx + y+1], this.mesh[(x+1)*this.elx + y+1], 4)
                    v_drawLine(this.mesh[x*this.elx + y], this.mesh[(x+1)*this.elx + y+1], 4)
                    v_drawLine(this.mesh[(x+1)*this.elx + y], this.mesh[x*this.elx + y+1], 4)
                } else {
                    ctx.fillStyle = this.color
                    ctx.strokeStyle = this.color
                    ctx.beginPath()
                    ctx.moveTo(this.mesh[x*this.elx + y].x, this.mesh[x*this.elx + y].y)
                    ctx.lineTo(this.mesh[(x+1)*this.elx + y].x, this.mesh[(x+1)*this.elx + y].y)
                    ctx.lineTo(this.mesh[(x+1)*this.elx + y+1].x, this.mesh[(x+1)*this.elx + y+1].y)
                    ctx.lineTo(this.mesh[(x+0)*this.elx + y+1].x, this.mesh[(x+0)*this.elx + y+1].y)
                    ctx.lineTo(this.mesh[x*this.elx + y].x, this.mesh[x*this.elx + y].y)
                    ctx.fill()
                    ctx.stroke()
                }
            }
        }
    }
}

let physics_bodies = []
let colors = [
    'red',
    'orange',
    'yellow',
    'lime',
    'green',
    'cyan',
    'blue',
    'magenta',
    'purple',
    'black',
    'grey',
    '#e6e6e6'
]
let debug_move = false;

function init() {
    canvas = document.getElementById('canvas')
    ctx = canvas.getContext('2d')
    setInterval(update, 25)
    render()

    //input
    window.addEventListener('mousedown', (e) => {
        let chosen_col = colors[Math.floor(Math.random()*colors.length)]
        createSoftBody(mouse.x, mouse.y, 196, 196, 6, 6, 0.1, chosen_col)
    })
    window.addEventListener('mouseup', (e) => {
        debug_move = false;
    })
    window.addEventListener('mousemove', (e) => {
        mouse = {x:e.clientX, y:e.clientY}
    })

    // createSoftBody(10, 10, 128, 128, 8, 8, 0.25, 'orange')
}
function update() {
    solveAllPhysics()
}

function createSoftBody(x, y, w, h, elements_x, elements_y, softness, color) {
    physics_bodies.push(new SoftBody(x, y, w, h, elements_x, elements_y, softness, color))
}

function solveAllPhysics() {
    for(let body of physics_bodies) {
        body.update()
    }
}

function render() {
    ctx.clearRect(0, 0, canvas.width, canvas.height)
    for(let body of physics_bodies) {
        body.render()
    }
    requestAnimationFrame(render)
}

function drawLine(x1, y1, x2, y2, thickness) {
    ctx.beginPath()
    ctx.moveTo(x1, y1)
    ctx.lineTo(x2, y2)
    ctx.lineWidth = 2;
    if(thickness != undefined) ctx.lineWidth = thickness;
    ctx.stroke()

}

function v_drawLine(v1, v2, thickness) {
    drawLine(v1.x, v1.y, v2.x, v2.y, thickness)
}

function v_add(v1, v2) {
    return {x: v1.x + v2.x, y: v1.y + v2.y}
}
function v_sub(v1, v2) {
    return {x: v1.x - v2.x, y: v1.y - v2.y}
}
function v_mul_n(v1, n) {
    return {x: v1.x * n, y: v1.y * n}
}
function v_div_n(v1, n) {
    return {x: v1.x / n, y: v1.y / n}
}
function v_abs(v1) {
    return {x: Math.abs(v1.x), y: Math.abs(v1.y)}
}
function v_length(v1) {
    return Math.hypot(v1.x, v1.y);
}

function n_clamp(x, min, max) {
    if(x < min) {
      return min;
    } else if(x > max) {
      return max;
    } else {
      return x;
    }
}

function v_distance(a, b) {
  //returns distance between 2 positions. Uses vectors as arguments
  let _a = a.x - b.x, _b = a.y - b.y;
  return Math.sqrt(_a*_a + _b*_b);
}