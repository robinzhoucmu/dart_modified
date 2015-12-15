#include "dart/dart.h"

const double default_shape_density = 1000; // kg/m^3
const double default_shape_height  = 0.03;  // m
const double default_shape_width   = 0.1; // m
const double default_skin_thickness = 1e-3; // m

//const double default_start_height = 0.4;  // m
const double default_start_height = 0.4;  // m

const double minimum_start_v = 2.5; // m/s
const double maximum_start_v = 4.0; // m/s
const double default_start_v = 3.5; // m/s

const double minimum_launch_angle = 30.0*M_PI/180.0; // rad
const double maximum_launch_angle = 70.0*M_PI/180.0; // rad
const double default_launch_angle = 45.0*M_PI/180.0; // rad

const double maximum_start_w = 6*M_PI; // rad/s
const double default_start_w = 3*M_PI;  // rad/s

const double default_damping_coefficient = 0.001;

const double default_ground_width = 2;
const double default_wall_thickness = 0.1;
const double default_wall_height = 1;
const double default_spawn_range = 0.9*default_ground_width/2;

const double default_restitution = 0.6;

using namespace dart::dynamics;
using namespace dart::simulation;

class MyWindow : public dart::gui::SimWindow {
 public:

  MyWindow(const WorldPtr& world, const SkeletonPtr& block) : 
    mOriginalBlock(block),
    mSkelCount(0) 
  {    
    setWorld(world);
    // world->getConstraintSolver()->setCollisionDetector(new dart::collision::FCLCollisionDetector());
  }

  void keyboard(unsigned char key, int x, int y) override
  {
    switch(key) {
      case '1':
        addObject(mOriginalBlock->clone());
        break;
      case 'd':
        if(mWorld->getNumSkeletons() > 2)
          removeSkeleton(mWorld->getSkeleton(2));
        std::cout << "Remaining objects: " << mWorld->getNumSkeletons()-2
                  << std::endl;
        break;

      default:
        SimWindow::keyboard(key, x, y);
    }
  }


  void drawSkels() override
  {
    // Make sure lighting is turned on and that polygons get filled in
    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    SimWindow::drawSkels();
  }

  void displayTimer(int _val) override
  {
    // We remove playback and baking, because we want to be able to add and
    // remove objects during runtime
    int numIter = mDisplayTimeout / (mWorld->getTimeStep() * 1000);
    if (mSimulating)
    {
      for (int i = 0; i < numIter; i++) {
	//printContactInfo();
	timeStepping();
      }
    }
    glutPostRedisplay();
    glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
  }
  
  void timeStepping() override 
  {
    // Step the simulation forward
    SimWindow::timeStepping();
  }
 protected:
  void printContactInfo() {
    dart::collision::CollisionDetector* detector = 
      mWorld->getConstraintSolver()->getCollisionDetector();
    detector->detectCollision(true, true);
    size_t collisionCount = detector->getNumContacts();
    for (size_t i = 0; i < collisionCount; ++i) {
      const dart::collision::Contact& contact = detector->getContact(i);
      // Print out contact information.
      std::cout << "Contact #" << i << std::endl;
      std::cout << "Point: "<< std::endl;
      std::cout <<  contact.point << std::endl;
      std::cout << "Normal: " << std::endl;
      std::cout << contact.normal << std::endl;
      std::cout << "Penetration Depth: " << std::endl;
      std::cout << contact.penetrationDepth << std::endl;
    }
  }
  /// Add an object to the world and toss it at the wall
  bool addObject(const SkeletonPtr& object)
  {
    // Set the starting position for the object
    Eigen::Vector6d positions(Eigen::Vector6d::Zero());

    positions[5] = default_start_height;

    object->getJoint(0)->setPositions(positions);

    // Add the object to the world
    object->setName(object->getName()+std::to_string(mSkelCount++));
    mWorld->addSkeleton(object);

    // Compute collisions
    dart::collision::CollisionDetector* detector =
        mWorld->getConstraintSolver()->getCollisionDetector();
    detector->detectCollision(true, true);

    // Look through the collisions to see if the new object would start in
    // collision with something
    bool collision = false;
    size_t collisionCount = detector->getNumContacts();
    for(size_t i = 0; i < collisionCount; ++i)
    {
      const dart::collision::Contact& contact = detector->getContact(i);
      if(contact.bodyNode1.lock()->getSkeleton() == object
         || contact.bodyNode2.lock()->getSkeleton() == object)
      {
        collision = true;
        break;
      }
    }

    // Refuse to add the object if it is in collision
    if(collision)
    {
      mWorld->removeSkeleton(object);
      std::cout << "The new object spawned in a collision. "
                << "It will not be added to the world." << std::endl;
      return false;
    }

    // Create reference frames for setting the initial velocity
    Eigen::Isometry3d centerTf(Eigen::Isometry3d::Identity());
    centerTf.translation() = object->getCOM();
    SimpleFrame center(Frame::World(), "center", centerTf);

    // Set the velocities of the reference frames so that we can easily give the
    // Skeleton the linear and angular velocities that we want
    double angle = default_launch_angle;
    double speed = default_start_v;
    double angular_speed = default_start_w;

    angle = -M_PI/4;
    angular_speed = 0.0;

    Eigen::Vector3d v = speed * Eigen::Vector3d(cos(angle), 0.0, sin(angle));
    Eigen::Vector3d w = angular_speed * Eigen::Vector3d::UnitY();
    center.setClassicDerivatives(v, w);

    SimpleFrame ref(&center, "root_reference");
    ref.setRelativeTransform(object->getBodyNode(0)->getTransform(&center));

    // Use the reference frames to set the velocity of the Skeleton's root
    object->getJoint(0)->setVelocities(ref.getSpatialVelocity());

    return true;
  }

  /// Remove a Skeleton and get rid of the constraint that was associated with
  /// it, if one existed
  void removeSkeleton(const SkeletonPtr& skel)
  {
    for(size_t i=0; i<mJointConstraints.size(); ++i)
    {
      dart::constraint::JointConstraint* constraint = mJointConstraints[i];
      if(constraint->getBodyNode1()->getSkeleton() == skel
         || constraint->getBodyNode2()->getSkeleton() == skel)
      {
        mWorld->getConstraintSolver()->removeConstraint(constraint);
        mJointConstraints.erase(mJointConstraints.begin()+i);
        delete constraint;
        break; // There should only be one constraint per skeleton
      }
    }

    mWorld->removeSkeleton(skel);
  }


  /// History of the active JointConstraints so that we can properly delete them
  /// when a Skeleton gets removed
  std::vector<dart::constraint::JointConstraint*> mJointConstraints;
  /// A blueprint Skeleton that we will use to spawn blocks.
  SkeletonPtr mOriginalBlock;
  /// Keep track of how many Skeletons we spawn to ensure we can give them all
  /// unique names
  size_t mSkelCount;
};

template<class JointType>
BodyNode* addRigidBody(const SkeletonPtr& chain, const std::string& name,
                       Shape::ShapeType type, BodyNode* parent = nullptr)
{
  // Set the Joint properties
  typename JointType::Properties properties;
  properties.mName = name+"_joint";
  if(parent)
  {
    // If the body has a parent, we should position the joint to be in the
    // middle of the centers of the two bodies
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = Eigen::Vector3d(0, 0, default_shape_height / 2.0);
    properties.mT_ParentBodyToJoint = tf;
    properties.mT_ChildBodyToJoint = tf.inverse();
  }

  // Create the Joint and Body pair
  BodyNode* bn = chain->createJointAndBodyNodePair<JointType>(
        parent, properties, BodyNode::Properties(name)).second;

  // Make the shape based on the requested Shape type
  ShapePtr shape;
  if(Shape::BOX == type)
  {
    shape = std::make_shared<BoxShape>(Eigen::Vector3d(
                                         default_shape_width,
                                         default_shape_width,
                                         default_shape_height));
  }
  else if(Shape::CYLINDER == type)
  {
    shape = std::make_shared<CylinderShape>(default_shape_width/2.0,
                                            default_shape_height);
  }
  else if(Shape::ELLIPSOID == type)
  {
    shape = std::make_shared<EllipsoidShape>(
          default_shape_height*Eigen::Vector3d::Ones());
  }

  bn->addVisualizationShape(shape);
  bn->addCollisionShape(shape);

  // Setup the inertia for the body
  Inertia inertia;
  double mass = default_shape_density * shape->getVolume();
  inertia.setMass(mass);
  inertia.setMoment(shape->computeInertia(mass));
  bn->setInertia(inertia);

  // Set the coefficient of restitution to make the body more bouncy
  bn->setRestitutionCoeff(default_restitution);

  // Set damping to make the simulation more stable
  if(parent)
  {
    Joint* joint = bn->getParentJoint();
    for(size_t i=0; i < joint->getNumDofs(); ++i)
      joint->getDof(i)->setDampingCoefficient(default_damping_coefficient);
  }

  return bn;
}

void setAllColors(const SkeletonPtr& object, const Eigen::Vector3d& color)
{
  // Set the color of all the shapes in the object
  for(size_t i=0; i < object->getNumBodyNodes(); ++i)
  {
    BodyNode* bn = object->getBodyNode(i);
    for(size_t j=0; j < bn->getNumVisualizationShapes(); ++j)
      bn->getVisualizationShape(j)->setColor(color);
  }
}


SkeletonPtr createBlock()
{
  SkeletonPtr block = Skeleton::create("rigid_block");
  // Give the ball a body
  addRigidBody<FreeJoint>(block, "rigid block", Shape::BOX);
  setAllColors(block, dart::Color::Red());

  return block;
}

SkeletonPtr createGround()
{
  SkeletonPtr ground = Skeleton::create("ground");

  BodyNode* bn = ground->createJointAndBodyNodePair<WeldJoint>().second;

  std::shared_ptr<BoxShape> shape = std::make_shared<BoxShape>(
        Eigen::Vector3d(default_ground_width, default_ground_width,
                        default_wall_thickness));
  shape->setColor(Eigen::Vector3d(1.0, 1.0, 1.0));

  bn->addCollisionShape(shape);
  bn->addVisualizationShape(shape);

  return ground;
}

SkeletonPtr createWall()
{
  SkeletonPtr wall = Skeleton::create("wall");

  BodyNode* bn = wall->createJointAndBodyNodePair<WeldJoint>().second;

  std::shared_ptr<BoxShape> shape = std::make_shared<BoxShape>(
        Eigen::Vector3d(default_wall_thickness, default_ground_width,
                        default_wall_height));
  shape->setColor(Eigen::Vector3d(0.8, 0.8, 0.8));

  bn->addCollisionShape(shape);
  bn->addVisualizationShape(shape);

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(
        (default_ground_width + default_wall_thickness)/2.0, 0.0,
        (default_wall_height  - default_wall_thickness)/2.0);
  bn->getParentJoint()->setTransformFromParentBodyNode(tf);

  bn->setRestitutionCoeff(0.2);

  return wall;
}

int main(int argc, char* argv[]) {
  WorldPtr world = std::make_shared<World>();
  world->addSkeleton(createGround());
  world->addSkeleton(createWall());
  
  MyWindow window(world, createBlock());
  std::cout << "space bar: simulation on/off" << std::endl;
  std::cout << "'1': toss a rigid ball" << std::endl;
  std::cout << "\n'd': delete the oldest object" << std::endl;
  
  glutInit(&argc, argv);
  window.initWindow(640, 480, "Collisions");
  glutMainLoop();
}
