// gta physics?

var debugMode = false;

$("#toggleDebug").click(function(e) {
  debugMode = !debugMode;
  if(!debugMode) {
    $("#debugCanvas").hide();
  } else {
    $("#debugCanvas").show();
  }
});

(function() {
  console.log("box2d: ", Box2D);

  var b2Vec2 = Box2D.Common.Math.b2Vec2;
  var b2AABB = Box2D.Collision.b2AABB;
  var b2BodyDef = Box2D.Dynamics.b2BodyDef;
  var b2Body = Box2D.Dynamics.b2Body;
  var b2FixtureDef = Box2D.Dynamics.b2FixtureDef;
  var b2Fixture = Box2D.Dynamics.b2Fixture;
  var b2World = Box2D.Dynamics.b2World;
  var b2MassData = Box2D.Collision.Shapes.b2MassData;
  var b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape;
  var b2CircleShape = Box2D.Collision.Shapes.b2CircleShape;
  var b2DebugDraw = Box2D.Dynamics.b2DebugDraw;
  var b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef;
  var b2PrismaticJointDef = Box2D.Dynamics.Joints.b2PrismaticJointDef;
  var b2Math = Box2D.Common.Math.b2Math;

  var MAX_STEER_ANGLE = Math.PI/3;
  var STEER_SPEED = 1.5;
  var SIDEWAYS_FRICTION_FORCE = 10;
  var HORSEPOWERS = 40;
  var CAR_STARTING_POS = new b2Vec2(15, 10);

  var leftRearWheelPosition = new b2Vec2(-1.5, 1.90);
  var rightRearWheelPosition = new b2Vec2(1.5, 1.9);
  var leftFrontWheelPosition = new b2Vec2(-1.5, -1.9);
  var rightFrontWheelPosition = new b2Vec2(1.5, -1.9);

  var engineSpeed = 0;
  var steeringAngle = 0;

  var worldScale = 10;

  var myWorld;

  var worldBox = new b2AABB();
  worldBox.lowerBound.Set(-100, -100);
  worldBox.upperBound.Set(100, 100);

  myWorld = new b2World(new b2Vec2(), true);

  var canvas = document.getElementById("canvas");
  var context = canvas.getContext("2d");



  myWorld.draw = function() {
    context.clearRect(0, 0, canvas.width, canvas.height);
    for(var b = myWorld.m_bodyList; b !== null; b = b.m_next) {
      if(b.GetUserData()) {
        var data = b.GetUserData();

        context.save();
        context.translate(b.GetPosition().x * worldScale, b.GetPosition().y * worldScale);
        context.rotate(b.GetAngle());
        context.drawImage(data.image, -((data.width * worldScale)/2), -((data.height * worldScale)/2), data.width*worldScale, data.height*worldScale);
        context.restore();
      }
    }
  };

  function createBox(width, height, pX, pY, type, data) {
    var bodyDef = new b2BodyDef();
    bodyDef.type = type;
    bodyDef.position.Set(pX, pY);
    bodyDef.userData = {image: data, width: width, height: height};

    var polygonShape = new b2PolygonShape();
    polygonShape.SetAsBox(width/2 ,height/2);

    var fixtureDef = new b2FixtureDef();
    fixtureDef.density = 1.0;
    fixtureDef.friction = 0.5;
    fixtureDef.restitution = 0.5;
    fixtureDef.shape = polygonShape;

    var body = myWorld.CreateBody(bodyDef);
    body.CreateFixture(fixtureDef);

    return body;
  }

  var crateImg = document.getElementById("crate");
  var carImg = document.getElementById("car1");
  var carImg2 = document.getElementById("car2");

  createBox(10, 10, 5, 20, b2Body.b2_staticBody, crateImg);
  createBox(10, 10, 25, 20, b2Body.b2_staticBody, crateImg);
  createBox(10, 10, 75, 20, b2Body.b2_staticBody, crateImg);
  createBox(10, 10, 25, 50, b2Body.b2_staticBody, crateImg);
  createBox(5, 5, 5, 5, b2Body.b2_staticBody, crateImg);

  function Tire(world, data) {
    this._maxForwardSpeed = 0;
    this._maxBackwardSpeed = 0;
    this._maxDriveForce = 0;

    var bodyDef = new b2BodyDef();
    bodyDef.type = b2Body.b2_dynamicBody;
    bodyDef.position = CAR_STARTING_POS.Copy();
    this._body = world.CreateBody(bodyDef);

    var shape = new b2PolygonShape();
    shape.SetAsBox( 0.5, 1.25 );
    this._body.CreateFixture2(shape, 1);//shape, density

    this._body.SetUserData(data);
  }

  Tire.prototype.setCharacteristics = function(maxForwardSpeed, maxBackwardSpeed, maxDriveForce) {
      this._maxForwardSpeed = maxForwardSpeed;
      this._maxBackwardSpeed = maxBackwardSpeed;
      this._maxDriveForce = maxDriveForce;
  };

  Tire.prototype.getLateralVelocity = function() {
      var self = this;
      var currentRightNormal = this._body.GetWorldVector(new b2Vec2(1, 0));
      var dot = b2Math.Dot( currentRightNormal, self._body.GetLinearVelocity() );
      currentRightNormal.Multiply(dot);
      return currentRightNormal;
  };

  Tire.prototype.getForwardVelocity = function() {
      var currentForwardNormal = this._body.GetWorldVector(new b2Vec2(0,1));
      var dot = b2Math.Dot( currentForwardNormal, this._body.GetLinearVelocity() );
      currentForwardNormal.Multiply(dot);
      return currentForwardNormal;
  };

  Tire.prototype.updateFriction = function() {
      //lateral linear velocity
      var maxLateralImpulse = 2.5;
      var impulse = this.getLateralVelocity().GetNegative();
      impulse.Multiply(this._body.GetMass());

      if(impulse.Length() === 0) return;

      if ( impulse.Length() > maxLateralImpulse )
          impulse.Multiply(maxLateralImpulse / impulse.Length());
      this._body.ApplyImpulse( impulse, this._body.GetWorldCenter() );

      //angular velocity
      var angularImpulse =  0.1 * this._body.GetInertia() * -(this._body.GetAngularVelocity());
      //console.log(">impulse", angularImpulse);
      this._body.ApplyAngularImpulse( angularImpulse );

      //forward linear velocity
      var currentForwardNormal = this.getForwardVelocity();
      var currentForwardSpeed = currentForwardNormal.Normalize();
      var dragForceMagnitude = -2 * currentForwardSpeed;
      currentForwardNormal.Multiply(dragForceMagnitude);
      this._body.ApplyForce( currentForwardNormal, this._body.GetWorldCenter() );
  };

  Tire.prototype.updateDrive = function(controlState) {

        //find desired speed
        var desiredSpeed = 0;
        if(controlState.UP === 1) {
          desiredSpeed = this._maxForwardSpeed;
        }
        if(controlState.DOWN === 1) {
          desiredSpeed = this._maxBackwardSpeed;
        }
        if(desiredSpeed === 0) return; // do nothing

        //find current speed in forward direction
        var currentForwardNormal = this._body.GetWorldVector(new b2Vec2(0,1) );
        var currentSpeed = b2Math.Dot( this.getForwardVelocity(), currentForwardNormal );

        //apply necessary force
        var force = 0;
        if ( desiredSpeed > currentSpeed )
            force = this._maxDriveForce;
        else if ( desiredSpeed < currentSpeed )
            force = -this._maxDriveForce;
        else
            return;

        currentForwardNormal.Multiply(force);
        this._body.ApplyForce( currentForwardNormal, this._body.GetWorldCenter() );
    };

  Car = function(world, data) {
    this._tires = [];

    var bodyDef = new b2BodyDef();
    bodyDef.type = b2Body.b2_dynamicBody;
    bodyDef.position = data.position ? data.position.Copy() : new b2Vec2();
    bodyDef.userData = {image: data.image, width: 8, height: 12};
    this._body = world.CreateBody(bodyDef);

    var shape = new b2PolygonShape();
    var vertices = [];
    vertices[0] = new b2Vec2( 1.5,   0);
    vertices[1] = new b2Vec2(   3, 2.5);
    vertices[2] = new b2Vec2( 2.8, 5.5);
    vertices[3] = new b2Vec2(   1,  10);
    vertices[4] = new b2Vec2(  -1,  10);
    vertices[5] = new b2Vec2(-2.8, 5.5);
    vertices[6] = new b2Vec2(  -3, 2.5);
    vertices[7] = new b2Vec2(-1.5,   0);
    //shape.SetAsArray(vertices);
    shape.SetAsBox(3, 6);
    this._body.CreateFixture2(shape, 0.1);//shape, density

    //prepare common joint parameters
    var jointDef= new b2RevoluteJointDef();
    jointDef.bodyA = this._body;
    jointDef.enableLimit = true;
    jointDef.lowerAngle = 0;
    jointDef.upperAngle = 0;
    jointDef.localAnchorB.SetZero();//center of tire

    var maxForwardSpeed = 250;
    var maxBackwardSpeed = -40;
    var backTireMaxDriveForce = 300;
    var frontTireMaxDriveForce = 500;
    var backTireMaxLateralImpulse = 8.5;
    var frontTireMaxLateralImpulse = 7.5;

    //back left tire
    var tire = new Tire(world);
    tire.setCharacteristics(maxForwardSpeed, maxBackwardSpeed, backTireMaxDriveForce, backTireMaxLateralImpulse);
    jointDef.bodyB = tire._body;
    jointDef.localAnchorA.Set( -3, -1.75 );
    world.CreateJoint( jointDef );
    this._tires.push(tire);

    //back right tire
    tire = new Tire(world);
    tire.setCharacteristics(maxForwardSpeed, maxBackwardSpeed, backTireMaxDriveForce, backTireMaxLateralImpulse);
    jointDef.bodyB = tire._body;
    jointDef.localAnchorA.Set( 3, -1.75 );
    world.CreateJoint( jointDef );
    this._tires.push(tire);

    //front left tire
    tire = new Tire(world);
    tire.setCharacteristics(maxForwardSpeed, maxBackwardSpeed, frontTireMaxDriveForce, frontTireMaxLateralImpulse);
    jointDef.bodyB = tire._body;
    jointDef.localAnchorA.Set( -3, 3.5 );
    this.flJoint = world.CreateJoint( jointDef );
    this._tires.push(tire);

    //front right tire
    tire = new Tire(world);
    tire.setCharacteristics(maxForwardSpeed, maxBackwardSpeed, frontTireMaxDriveForce, frontTireMaxLateralImpulse);
    jointDef.bodyB = tire._body;
    jointDef.localAnchorA.Set( 3, 3.5 );
    this.frJoint = world.CreateJoint( jointDef );
    this._tires.push(tire);
  };

var DEGTORAD = 0.0174532925199432957;
var RADTODEG = 57.295779513082320876;

  Car.prototype.update = function(controlState) {
      for (var i = 0, len=this._tires.length; i < len; i++) {
          this._tires[i].updateFriction();
          this._tires[i].updateDrive(controlState);
      }

      //control steering
      var lockAngle = 35 * DEGTORAD;
      var turnSpeedPerSec = 160 * DEGTORAD;//from lock to lock in 0.5 sec
      var turnPerTimeStep = turnSpeedPerSec / 60.0;
      var desiredAngle = 0;

      if(controlState.RIGHT === 1) {
        desiredAngle = lockAngle;
      }

      if(controlState.LEFT === 1) {
        desiredAngle = -lockAngle;
      }

      var angleNow = this.flJoint.GetJointAngle();
      var angleToTurn = desiredAngle - angleNow;
      angleToTurn = b2Math.Clamp( angleToTurn, -turnPerTimeStep, turnPerTimeStep );
      var newAngle = angleNow + angleToTurn;
      this.flJoint.SetLimits( newAngle, newAngle );
      this.frJoint.SetLimits( newAngle, newAngle );
  };

  var myCar = new Car(myWorld, {image: carImg, position: CAR_STARTING_POS});
  var AnotherCar = new Car(myWorld, {image: carImg2, position: new b2Vec2(80, 30)});

  //setup debug draw
  var debugDraw = new b2DebugDraw();
  debugDraw.SetSprite(document.getElementById("debugCanvas").getContext("2d"));
  debugDraw.SetDrawScale(worldScale);
  debugDraw.SetFillAlpha(0.3);
  debugDraw.SetLineThickness(1.0);
  debugDraw.SetFlags(b2DebugDraw.e_shapeBit | b2DebugDraw.e_jointBit | b2DebugDraw.e_centerOfMassBit);
  myWorld.SetDebugDraw(debugDraw);

  console.log("DEBUGDRAW", debugDraw);

  window.setInterval(tick, 1000 / 60);
  //window.requestAnimationFrame(tick);
  document.onkeydown = keyPressed_handler;
  document.onkeyup = keyReleased_handler;

  function tick() {
    myCar.update(ControlState);
    AnotherCar.update({UP:0,DOWN:0,LEFT:0,RIGHT:0});

    myWorld.Step(1 / 60, 10, 10);

    myWorld.draw();

    if(debugMode)
      myWorld.DrawDebugData();

    myWorld.ClearForces();
  }

  var Keyboard = {
    UP: 38,
    DOWN: 40,
    LEFT: 37,
    RIGHT: 39
  };

  var ControlState = {
    UP: 0,
    DOWN: 0,
    LEFT: 0,
    RIGHT: 0
  };

  function keyPressed_handler(e) {
    if(e.keyCode == Keyboard.UP) {
      ControlState.UP = 1;
    }
    if(e.keyCode == Keyboard.DOWN) {
      ControlState.DOWN = 1;
    }
    if(e.keyCode == Keyboard.RIGHT) {
      ControlState.RIGHT = 1;
    }
    if(e.keyCode == Keyboard.LEFT) {
      ControlState.LEFT = 1;
    }
  }

  function keyReleased_handler(e){
    if(e.keyCode == Keyboard.UP) ControlState.UP = 0;
    if(e.keyCode == Keyboard.DOWN) ControlState.DOWN = 0;
    if(e.keyCode == Keyboard.LEFT) ControlState.LEFT = 0;
    if(e.keyCode == Keyboard.RIGHT) ControlState.RIGHT = 0;
  }

})();