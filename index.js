//vars
let canvas
let ctx
let substep_cnt = 1

class SoftBody{
    constructor(ofsx, ofsy, w, h, elemx, elemy) {
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
    }
    init() {

    }
    update() {
        //set pos to be previous before we change it

        //update delta time
        const delta_time = (Date.now()-this.prev)/1000

        //solve 
        for(let sub=0; sub<substep_cnt; sub++) {
            //step A - apply transformation
            for(let x = 0; x < this.elx; x++) {
                for(let y = 0; y < this.ely; y++) {
                    //calculate gravity
                    this.mesh_vel[x*this.elx + y] = v_add(this.mesh_vel[x*this.elx + y], v_mul_n({x:0, y:0.98}, delta_time))

                    //save previous pos
                    this.mesh_prev[x*this.elx + y] = {x: this.mesh[x*this.elx + y].x, y:this.mesh[x*this.elx + y].y}
    
                    //add saved velocity to current point
                    this.mesh[x*this.elx + y] = v_add(this.mesh[x*this.elx + y], v_mul_n(this.mesh_vel[x*this.elx + y], delta_time))
    
                }
            }
            //step B - apply constraints

            //ground
            for(let x = 0; x < this.elx; x++) {
                for(let y = 0; y < this.ely; y++) {
                    if(this.mesh[x*this.elx + y].y > canvas.height) {
                        this.mesh[x*this.elx + y].y = canvas.height
                    }
                }
            }

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
                        dpos = v_mul_n(dpos, 1/v_length(dpos))

                        let delta1 = v_mul_n(dpos, ddist*0.5)
                        let delta2 = v_mul_n(dpos, -ddist*0.5)

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
                        const restl = this.w / this.elx

                        //delta calculations
                        let ddist = v_length(v_sub(x1, x2)) - restl
                        let dpos = v_sub(x2, x1)
                        dpos = v_mul_n(dpos, 1/v_length(dpos))

                        let delta1 = v_mul_n(dpos, ddist*0.5)
                        let delta2 = v_mul_n(dpos, -ddist*0.5)

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
                        let ddist = v_length(v_sub(x1, x2)) - restl
                        let dpos = v_sub(x2, x1)
                        dpos = v_mul_n(dpos, 1/v_length(dpos))

                        let delta1 = v_mul_n(dpos, ddist*0.5)
                        let delta2 = v_mul_n(dpos, -ddist*0.5)

                        //apply delta offsets
                        this.mesh[x*this.elx + y] = v_add(this.mesh[x*this.elx + y], delta1)
                        this.mesh[(x+1)*this.elx + y+1] = v_add(this.mesh[(x+1)*this.elx + y+1], delta2)
                    }
                }
            }

            //step C - XPBD formula
            for(let x = 0; x < this.elx; x++) {
                for(let y = 0; y < this.ely; y++) {
                    this.mesh_vel[x*this.elx + y] = v_div_n(v_sub(this.mesh[x*this.elx + y], this.mesh_prev[x*this.elx + y]), delta_time)
                }
            }
        }
    }
    render() {
        for(let x = 0; x < this.elx-1; x++) {
            for(let y = 0; y < this.ely-1; y++) {
                v_drawLine(this.mesh[x*this.elx + y], this.mesh[x*this.elx + y+1], 4)
                v_drawLine(this.mesh[x*this.elx + y], this.mesh[(x+1)*this.elx + y], 4)
                v_drawLine(this.mesh[(x+1)*this.elx + y], this.mesh[(x+1)*this.elx + y+1], 4)
                v_drawLine(this.mesh[x*this.elx + y+1], this.mesh[(x+1)*this.elx + y+1], 4)
            }
        }
    }
}

let physics_bodies = []


function init() {
    canvas = document.getElementById('canvas')
    ctx = canvas.getContext('2d')
    setInterval(update, 25)
    render()

    //input
    window.addEventListener('mousedown', (e) => {
    })
    window.addEventListener('mouseup', (e) => {
        
    })
    window.addEventListener('mousemove', (e) => {
        
    })

    createSoftBody(10, 10, 256, 256, 8, 8)
}
function update() {
    solveAllPhysics()
}

function createSoftBody(x, y, w, h, elements_x, elements_y) {
    physics_bodies.push(new SoftBody(x, y, w, h, elements_x, elements_y))
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
    return Math.sqrt(v1.x*v1.x + v1.y*v1.y)
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
  return Math.sqrt(Math.pow(_a, 2) + Math.pow(_b, 2));
}