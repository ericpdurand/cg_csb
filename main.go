package main

import "fmt"
import "os"
import "math"

const impulse = 120

var (
    n int
    ncp int
    myPods [2]pod
    hisPods [2]pod
    checkpoints []checkpoint
    turns [4]int
    lastTarget [4]int
    canBoost [4]bool
    log bool
)
    
type Entity struct {
	x, y, vx, vy, r, m float64
}

func (e * Entity)clone() Entity {
    return Entity{e.x,e.y,e.vx,e.vy,e.r,e.m}    
}

type Vect struct {
    x, y float64
}

type pod struct {
    Entity
    angle float64
    canboost bool
    canthrust int
    targetcp int
    id int
}

func (p *pod)clone() pod {
    return pod{p.Entity.clone(),p.angle,p.canboost,p.canthrust,p.targetcp,p.id}    
}

type checkpoint struct {
    Entity
    id int
    dToNext float64
    dToEnd []float64
    vToNext Vect
}

type move struct {
    tx,ty float64
    thrust int
}

type collision struct {
    pod1,pod2 *pod
    ratio float64
}

func (m *move)print(){
    if m.thrust == 650 {
        fmt.Fprintf(os.Stdout, "%v %v BOOST\n",int(m.tx),int(m.ty))
    } else {
        fmt.Fprintf(os.Stdout, "%v %v %v\n",int(m.tx),int(m.ty),m.thrust)
    }
}

func main() {
    n = 3
    ncp = 3
    fmt.Fprintln(os.Stderr, "Turns, checkpoints",n, ncp)
    checkpoints = make([]checkpoint,ncp)
    var x,y float64
    x = 3000
    y = 3000 
    checkpoints[0] = checkpoint{Entity{x,y,0,0,600,0},0,0,make([]float64,n),Vect{}}
    x = 10000
    y = 3000
    checkpoints[1] = checkpoint{Entity{x,y,0,0,600,0},1,0,make([]float64,n),Vect{}}
    x = 7000
    y = 7000
    checkpoints[2] = checkpoint{Entity{x,y,0,0,600,0},2,0,make([]float64,n),Vect{}}   
    //Compute distances
    total := 0.0
    for i:= range checkpoints{
        v := Vect{}
        if i < len(checkpoints)-1 {
            checkpoints[i].dToNext = dist(checkpoints[i].Entity,checkpoints[i+1].Entity)
            total += checkpoints[i].dToNext
            v = Vect{checkpoints[i+1].x-checkpoints[i].x,checkpoints[i+1].y-checkpoints[i].y}
        } else {
            checkpoints[i].dToNext = dist(checkpoints[i].Entity,checkpoints[0].Entity)
            total += checkpoints[i].dToNext
            v = Vect{checkpoints[0].x-checkpoints[i].x,checkpoints[0].y-checkpoints[i].y}
        }
        (&v).normalize()
        checkpoints[i].vToNext=v
    }
    for i:= range checkpoints{
        distFindeTour := 0.0
        for k:=i;k<len(checkpoints);k++ {
            distFindeTour += checkpoints[k].dToNext
        }
        for j:=0;j<n;j++{
            checkpoints[i].dToEnd[j] = total * float64(n-j-1) +distFindeTour
        }
        //fmt.Fprintln(os.Stderr, "-- CP",checkpoints[i])
    }
    //Init the turns and last Target and Boost
    for i:=0;i<4;i++ {
        turns[i] = 0
        lastTarget[i] = 1
        canBoost[i] = true
    }
    
            var vx,vy, angle float64
            var nextCheckPointId int
            //fmt.Scan(&x,&y,&vx,&vy,&angle,&nextCheckPointId)
            x=7574
	    y=6930
            vx=-356
            vy=158
            angle=180
            nextCheckPointId=0	
            myPods[0] = pod{Entity{x,y,vx,vy,400,1},normalizeAngle(angle),true,0,nextCheckPointId,0}
            if nextCheckPointId != lastTarget[0] {
                if lastTarget[0] == 0 && nextCheckPointId == 1 {
                    turns[0] +=1
                }
                lastTarget[0] = nextCheckPointId
            }
            fmt.Fprintln(os.Stderr, "myPod", 0, myPods[0] ,lastTarget[0], turns[0])

            x=10716
            y=3340
            vx=-250
            vy=-247
            angle=-171
            nextCheckPointId=1
            myPods[1] = pod{Entity{x,y,vx,vy,400,1},normalizeAngle(angle),true,0,nextCheckPointId,1}
            if nextCheckPointId != lastTarget[1] {
                if lastTarget[1] == 0 && nextCheckPointId == 1 {
                    turns[1] +=1
                }
                lastTarget[1] = nextCheckPointId
            }
            fmt.Fprintln(os.Stderr, "myPod", 1, myPods[1] ,lastTarget[1], turns[1])
            
            x=5218
            y=5528
            vx=-331
            vy=-255
            angle=-123
            nextCheckPointId=0 
            hisPods[0] = pod{Entity{x,y,vx,vy,400,1},normalizeAngle(angle),true,0,nextCheckPointId,2}
            if nextCheckPointId != lastTarget[2] {
                if lastTarget[2] == 0 && nextCheckPointId == 1 {
                    turns[2] +=1
                }
                lastTarget[2] = nextCheckPointId
            }
            fmt.Fprintln(os.Stderr, "hisPod", 0, hisPods[0] ,lastTarget[2], turns[2])
             
            x=6618
            y=6638
            vx=-389
            vy=75
            angle=-123
            nextCheckPointId=0
            hisPods[1] = pod{Entity{x,y,vx,vy,400,1},normalizeAngle(angle),true,0,nextCheckPointId,3}
            if nextCheckPointId != lastTarget[3] {
                if lastTarget[3] == 0 && nextCheckPointId == 1 {
                    turns[3] +=1
                }
                lastTarget[3] = nextCheckPointId
            }
            fmt.Fprintln(os.Stderr, "hisPod", 1, hisPods[1] ,lastTarget[3], turns[3])

        fmt.Fprintln(os.Stderr, "Situation: ", evaluateSituation(myPods,hisPods))
        results := turnStrategy(myPods,hisPods, 1)
        for i,m := range results{
            if m.thrust == 650 {
                canBoost[myPods[i].id] = false
            }
            m.print()
        }
}

func turnStrategy(myPods,hisPods [2]pod,turns int) [2]move {
    result := [2]move{}
    bestScore := math.MinInt32
    myMoves := basicStrategy(myPods)
    hisMoves := basicStrategy(hisPods)
    moves := getMoves(myPods, myMoves)
    for _,m := range moves {
        newMyPods, newHisPods := simulateTurn(myPods, hisPods, m, hisMoves)
        //fmt.Fprintf(os.Stderr, "New MyPods %v\n", newMyPods )
        //fmt.Fprintf(os.Stderr, "New HisPods %v\n", newHisPods )
        score := evaluateSituation(newMyPods,newHisPods)
        //fmt.Fprintf(os.Stderr, "Score: %v, %v \n", score,m )
        if score > bestScore {
            bestScore = score
            result = m
            fmt.Fprintf(os.Stderr, "Best! %v, %v\n",m,score)
        }
    }
    return result
}

func simulateTurn(myPods, hisPods [2]pod, myMoves, hisMoves [2]move) (newMyPods, newHisPods [2]pod) {
    //Perform rotation (limit to 18 degrees except for 1st round)
    newMyPods = [2]pod{myPods[0].clone(), myPods[1].clone()}
    newHisPods = [2]pod{hisPods[0].clone(), hisPods[1].clone()}
    //fmt.Fprintf(os.Stderr, "--NewMyPod %v\n ", newMyPods[0] )
    rotate(&(newMyPods[0]),myMoves[0])
    rotate(&(newMyPods[1]),myMoves[1])
    rotate(&(newHisPods[0]),hisMoves[0])
    rotate(&(newHisPods[1]),hisMoves[1])
    //fmt.Fprintf(os.Stderr, "--Rotate %v\n", newMyPods[0] )
    //Apply acceleration. Pods' facing vector is multiplied by thrust, then added to speed vector
    accelerate(&(newMyPods[0]),myMoves[0])
    accelerate(&(newMyPods[1]),myMoves[1])
    accelerate(&(newHisPods[0]),hisMoves[0])
    accelerate(&(newHisPods[1]),hisMoves[1])
    //fmt.Fprintf(os.Stderr, "--Accelerate %v\n", newMyPods[0] )
    //Movement. Speed added to position. If collision, rebound, with elastic impulse of 120
    computeCollisions(&newMyPods,&newHisPods)
    //fmt.Fprintf(os.Stderr, "--Moves %v\n", newMyPods[0] )
    //Friction
    for i := range newMyPods{
        friction(&(newMyPods[i]))
        round(&(newMyPods[i]))
    }
    for i := range newHisPods{
        friction(&(newHisPods[i]))
        round(&(newHisPods[i]))
    }
    //fmt.Fprintf(os.Stderr, "--Friction-Round %v\n", newMyPods[0] )
    //Speed values are truncated and positions rounded to nearest int.
    return newMyPods, newHisPods    
}

func getMoves(myPods [2]pod, myMoves [2]move) ([][2]move) {
    result := make([][2]move,0)
    result = append(result,myMoves)
    Move1 := make([]move,10)
    Move2 := make([]move,10)
    angles := []float64{18.0,5,0,-5,-18.0}
    for i,a := range angles {
        x,y := polarToXY(1000,normalizeAngle(myPods[0].angle+a))
        x2,y2 :=  polarToXY(1000,normalizeAngle(myPods[1].angle+a))
        Move1[2*i] = move{myPods[0].x + x,myPods[0].y + y,100}
        Move1[2*i+1] = move{myPods[0].x + x,myPods[0].y + y,1}
        //fmt.Fprintf(os.Stderr," Moves: %v, %v\n",Move1[2*i],Move1[2*i+1])
        Move2[2*i] = move{myPods[1].x + x2,myPods[1].y + y2,100}
        Move2[2*i+1] = move{myPods[1].x + x2,myPods[1].y + y2,1}
        //fmt.Fprintf(os.Stderr," Moves: %v, %v\n",Move2[2*i],Move2[2*i+1])
    }
    for _,m1 := range Move1 {
        for _,m2 := range Move2 {
            //fmt.Fprintf(os.Stderr," Moves: %v, %v\n",m1,m2)
            result = append(result, [2]move{m1,m2})
        }
    }
    //fmt.Fprintf(os.Stderr," Moves: %v\n",result)
    return result
}

func computeCollisions(myPods,hisPods *[2]pod) {
    moved := [4]bool{}
    coll := make([]collision,0)
    log = true
    c1,r1 := isCollision(myPods[0].Entity, myPods[1].Entity)
    if c1 {
        coll = append(coll,collision{&(myPods[0]), &(myPods[1]), r1})
    }
    log = false
    c2,r2 := isCollision(myPods[0].Entity, hisPods[0].Entity)
    if c2 {
        coll = append(coll,collision{&myPods[0], &hisPods[0], r2})
    }
    c3,r3 := isCollision(myPods[0].Entity, hisPods[1].Entity)
    if c3 {
        coll = append(coll,collision{&myPods[0], &hisPods[1], r3})
    }
    c4,r4 := isCollision(myPods[1].Entity, hisPods[0].Entity)
    if c4 {
        coll = append(coll,collision{&myPods[1], &hisPods[0], r4})
    }
    c5,r5 := isCollision(myPods[1].Entity, hisPods[1].Entity)
    if c5 {
        coll = append(coll,collision{&myPods[1], &hisPods[1], r5})
    }
    c6,r6 := isCollision(hisPods[0].Entity, hisPods[1].Entity)
    if c6 {
        coll = append(coll,collision{&hisPods[0], &hisPods[1], r6})
    }
    if len(coll) > 0 {
        //fmt.Fprintf(os.Stderr, "--Collision! %v %v %v\n", coll[0].pod1.Entity,coll[0].pod2.Entity,coll[0].ratio )
        computeMove(&(coll[0].pod1.Entity), &(coll[0].pod2.Entity),coll[0].ratio,true)
        moved[coll[0].pod1.id] = true
        moved[coll[0].pod2.id] = true
        //fmt.Fprintf(os.Stderr, "-- After Collision! %v %v %v\n", coll[0].pod1.Entity,coll[0].pod2.Entity,coll[0].ratio )
    }
    for i := range myPods {
        if !moved[myPods[i].id] {
            myPods[i].x = myPods[i].x + myPods[i].vx
            myPods[i].y = myPods[i].y + myPods[i].vy
        }
        if !moved[hisPods[i].id] {
            hisPods[i].x = hisPods[i].x + hisPods[i].vx
            hisPods[i].y = hisPods[i].y + hisPods[i].vy
        }
    }
    //Now check the collisions with checkpoints
    for i,p := range myPods {
        if b,_ := isCollision(p.Entity,checkpoints[p.targetcp].Entity);b{
            myPods[i].targetcp = nextCP(myPods[i].targetcp)
        }
    }
    for i,p := range hisPods {
        if b,_ := isCollision(p.Entity,checkpoints[p.targetcp].Entity);b{
            hisPods[i].targetcp = nextCP(hisPods[i].targetcp)
        }
    }
}

func basicStrategy(myPods [2]pod) [2]move {
    result := [2]move{}
    for i,p:= range myPods {
        tangle := angle(p.x,p.y,checkpoints[p.targetcp].x, checkpoints[p.targetcp].y)
        diffAngle := tangle - p.angle 
        if p.angle == -1 {
            diffAngle = 0
        }
        dist := dist(p.Entity,checkpoints[p.targetcp].Entity)
        //fmt.Fprintln(os.Stderr, "--- Pod", diffAngle,dist )
        thrust := basicAngleStrategyWithBoost2(dist, diffAngle)
        if thrust == 650 && !canBoost[p.id] {
            thrust = 100
        }
        tx,ty := computeNewTarget(p.x,p.y,checkpoints[p.targetcp].x, checkpoints[p.targetcp].y,dist,diffAngle)
        result[i] = move{tx,ty,thrust}
    }
    return result
}

func rotate(t *pod, m move) {
    rot := angle(t.x,t.y,m.tx,m.ty)
    if ((rot-t.angle)>18) && (t.vx!=0) && (t.vy!=0) {
        rot = t.angle+18
    } else if ((rot-t.angle)< -18) && (t.vx!=0) && (t.vy!=0) {
        rot = t.angle-18
    }
    t.angle = normalizeAngle(rot)
}

func accelerate(t *pod, m move) {
    x,y := polarToXY(float64(m.thrust), t.angle)
    t.vx = t.vx + x
    t.vy = t.vy + y
} 

func friction(p *pod) {
    p.vx = 0.85 * p.vx
    p.vy = 0.85 * p.vy
}

func round(p *pod) {
    p.vx = math.Trunc(p.vx)
    p.vy = math.Trunc(p.vy)
    p.x = roundF(p.x)
    p.y = roundF(p.y)
}

func angle(x,y,tx,ty float64) float64 {
    return 180*math.Atan2(ty-y,tx-x)/math.Pi
}

func roundF(x float64) float64 {
    if (math.Ceil(x)-x) < (x-math.Floor(x)) { return math.Ceil(x) }
    return math.Floor(x)
}

func normalizeAngle(angle float64) float64 {
    if angle >180 {
        angle = angle -360    
    }
    return angle
}

func basicAngleStrategyWithBoost2(nextCheckpointDist float64, nextCheckpointAngle float64)(thrust int) {
    if nextCheckpointAngle > 90 || nextCheckpointAngle < -90 {
        if nextCheckpointDist > 3000 {
            thrust = 10
        } else {
            thrust = 1
        }
    } else if (nextCheckpointAngle < -10 || nextCheckpointAngle > 10) && nextCheckpointDist < 2000{
        thrust = 50
    } else if (nextCheckpointAngle < -45 || nextCheckpointAngle > 45) {
        thrust = 200 - 100*int(math.Abs(float64(nextCheckpointAngle))/float64(45))
    } else {
        thrust = 100
    }
    if nextCheckpointAngle < 2 && nextCheckpointAngle > -2 &&  nextCheckpointDist > 3000 {
        thrust = 650
    }
    return thrust
}

func evaluateSituation(myPods [2]pod, hisPods[2]pod) int {
    result := 0
    myBest := 0
    hisBest := 0
    //Get each Best situation
    for i:=0;i<2;i++{
        newBest := evaluatePod(myPods[i],true)
        //if myBest == 0 || newBest > myBest{
        //    myBest = newBest
        //}
        myBest += newBest
        newBest = evaluatePod(hisPods[i],false)
        if hisBest == 0 || newBest > hisBest{
            hisBest = newBest
        } 
    }
    result = myBest - hisBest
    return result
}

func evaluatePod(p pod,log bool) int {
    s := int(distP(0,0,p.vx,p.vy))
    toCP := int(distP(p.x,p.y,checkpoints[p.targetcp].x,checkpoints[p.targetcp].y))
    toEnd := int(checkpoints[p.targetcp].dToEnd[turns[p.id]])
    if p.targetcp == 0 {
     toEnd = int(checkpoints[p.targetcp].dToEnd[turns[p.id]+1])
    }
    //v := Vect{checkpoints[p.targetcp].x-p.x,checkpoints[p.targetcp].y-p.y}
    //(&v).normalize()
    //v.x = 100*v.x
    //v.y = 100*v.y
    //d := int(dot(v,checkpoints[p.targetcp].vToNext))
    //v2 := Vect{checkpoints[p.targetcp].x-p.x,checkpoints[p.targetcp].y-p.y}
    //(&v2).normalize()
    //dot2 := int(dot(v,v2))
    result := - toCP - toEnd
   // result := s - toCP - toEnd +d+dot2
    //result := s - toEnd + dot
    if log {fmt.Fprintf(os.Stderr, "--Eva POD: %v,%v,%v, %v, %v\n", p, s,toCP, toEnd, result)}
    return result
}

func computeNewTarget(x,y,nx,ny,dist,angle float64) (tx,ty float64){
    if angle > -45 && angle < 45 {
        curAngle := math.Atan2(ny-y,nx-x)/math.Pi*180
        newAngle := curAngle+angle
        tx, ty =  polarToXY(dist,newAngle)
        tx = x+tx
        ty = y+ty
        //fmt.Fprintln(os.Stderr, "---- ", tx, ty, angle, curAngle, newAngle )
        return tx, ty
    }
    return nx, ny
}

func polarToXY(norm,angle float64) (x,y float64){
    x = norm*math.Cos(angle/180*math.Pi)
    y = norm*math.Sin(angle/180*math.Pi)
    return x,y
}

func (v *Vect) normalize() {
        norm := distP(0, 0, v.x, v.y)
        if norm > 0 {
                v.x = v.x / norm
                v.y = v.y / norm
        }
}

func dist2(e1, e2 Entity) float64 {
	return (e1.x-e2.x)*(e1.x-e2.x) + (e1.y-e2.y)*(e1.y-e2.y)
}

func dist(e1, e2 Entity) float64 {
	return math.Sqrt(dist2(e1, e2))
}

func dist2P(x1, y1, x2, y2 float64) float64 {
        return (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)
}

func distP(x1, y1, x2, y2 float64) float64 {
        return math.Sqrt(dist2P(x1, y1, x2, y2))
} 

func dot(v1, v2 Vect) float64 {
        return v1.x*v2.x + v1.y*v2.y
}

func isCollision(e1, e2 Entity) (isColl bool, result float64) {
        // If speed norm is less than dist less radiuses, then no Collision
        sNorm := distP(e1.vx, e1.vy, e2.vx, e2.vy)
        //if log {fmt.Fprintf(os.Stderr,"sNorm: %v \n", sNorm)}
        if sNorm < dist(e1, e2)-float64(e1.r+e2.r) {
                return false, 0
        }
        // Normalize V -> N
        N := Vect{e1.vx - e2.vx, e1.vy - e2.vy}
        N.normalize()
        //if log {fmt.Fprintf(os.Stderr,"N: %v \n", N)}
        // Find vector C between centers
        C := Vect{e2.x - e1.x, e2.y - e1.y}
        // Compute product
        lengthC := distP(0, 0, C.x, C.y)
        D := dot(N, C)
        //if log {fmt.Fprintf(os.Stderr,"D, lengthC: %v %v \n", D, lengthC)}
        // if direction is opposite, no Collision
        if D <= 0 {
                return false, 0
        }
        //Get F length
        F := lengthC*lengthC - D*D
        sumR2 := (e1.r + e2.r) * (e1.r + e2.r)
        //if log {fmt.Fprintf(os.Stderr,"F, sumR2: %v %v \n", F, sumR2)}
        if F >= sumR2 {
                return false, 0
        }
        T := sumR2 - F
        if T < 0 {
                return false, 0
        }
        // Dist to travel
        dist := D - math.Sqrt(T)
        //if log {fmt.Fprintf(os.Stderr,"dist: %v \n", dist)}
        if sNorm < dist {
                return false, 0
        }
        // Compute the ratio
        ratio := dist / sNorm
        //if log {fmt.Fprintf(os.Stderr,"ratio: %v \n", ratio)}
        return true, ratio
}

func computeMove(e1, e2 *Entity, ratio float64, full bool) {
        // First get the things in contact
        e1.x = e1.x + ratio*e1.vx
        e1.y = e1.y + ratio*e1.vy
        e2.x = e2.x + ratio*e2.vx
        e2.y = e2.y + ratio*e2.vy
        // Compute the new speed vectors
        N := Vect{e2.x - e1.x, e2.y - e1.y}
        //fmt.Fprintf(os.Stderr,"N: %v \n", N)
        N.normalize()
        v1 := Vect{e1.vx, e1.vy}
        v2 := Vect{e2.vx, e2.vy}
        a1 := dot(v1, N)
        a2 := dot(v2, N)
        //fmt.Fprintf(os.Stderr,"a1: %v \n", a1)
        //fmt.Fprintf(os.Stderr,"a2: %v \n", a2)
        //Compute Optimized param
        optim :=  (a1 - a2) / (e1.m + e2.m)
        nv1 := Vect{v1.x - optim*e2.m*N.x, v1.y - optim*e2.m*N.y}
        nv2 := Vect{v2.x + optim*e1.m*N.x, v2.y + optim*e1.m*N.y}
        //Check the impulse
        i := Vect{optim*e2.m*e1.m*N.x, optim*e2.m*e1.m*N.y}
        iNorm := distP(0,0,i.x,i.y)
        if iNorm < impulse {
            i.x = i.x*120/iNorm
            i.y = i.y*120/iNorm
        }
        //apply again???
        nv1.x -= i.x/e1.m
        nv1.y -= i.y/e1.m
        nv2.x += i.x/e2.m
        nv2.y += i.y/e2.m
        
        if full {
            e1.x = e1.x + (1-ratio)*nv1.x
            e1.y = e1.y + (1-ratio)*nv1.y
            e2.x = e2.x + (1-ratio)*nv2.x
            e2.y = e2.y + (1-ratio)*nv2.y
        } 
        e1.vx = nv1.x
        e1.vy = nv1.y
        e2.vx = nv2.x
        e2.vy = nv2.y
}

func nextCP(id int) int {
    if id == len(checkpoints)-1 { return 0 }
    return id+1
}


