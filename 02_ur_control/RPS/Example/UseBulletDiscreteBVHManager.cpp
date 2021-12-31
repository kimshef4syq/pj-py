#include <RVS/CollisionChecker/BulletCollisionChecker/Bullet/BulletDiscreteBVHManager.h>
using namespace RVS;

bool ContactAllowedFn(const std::string &name1, const std::string &name2)
{
    if (name1 == "box1_link" && name2 == "box3_link") {
        return true;
    }
    else if (name1 == "box3_link" && name2 == "box1_link") {
        return true;
    }
    else {
        return false;
    }
}

int main()
{
    BulletDiscreteBVHManager checker;

    std::shared_ptr<Box> box1 = std::make_shared<Box>(1, 1, 1);
    SE3d box1_pose(-0.5, -0.5, 0.5, 0, 0, 0, 1);
    checker.addCollisionObject("box1_link", 0, box1, box1_pose);

    std::shared_ptr<Box> box2 = std::make_shared<Box>(1, 1, 1);
    SE3d box2_pose(0.501, 0.5, 0.5, 0, 0, 0, 1);
    checker.addCollisionObject("box2_link", 0, box2, box2_pose);

    std::shared_ptr<Box> box3 = std::make_shared<Box>(1, 1, 1);
    SE3d box3_pose(0.34, 0.5, 0.5, 0, 0, 0, 1);
    checker.addCollisionObject("box3_link", 0, box3, box3_pose);

    checker.setCollisionMarginData(CollisionMarginData(0.05));
    // checker.setIsContactAllowedFn(ContactAllowedFn);

    ContactResultMap result;
    ContactRequest request(ContactTestType::FIRST);
    checker.contactTest(result, request);

    ContactResultVector result_vector;
    flattenResults(std::move(result), result_vector);

    if (result_vector.empty()) {
        std::cout << "No Collision" << std::endl;
        std::cout << "Distance: " << result_vector[0].distance << std::endl;

        std::cout << "Link: " << result_vector[0].link_names[0]
                  << " nearest point: " << result_vector[0].nearest_points[0]
                  << std::endl;
        std::cout << "Link: " << result_vector[0].link_names[1]
                  << " nearest point: " << result_vector[0].nearest_points[1]
                  << std::endl;
        std::cout << "Direction to move Link  "
                  << result_vector[0].link_names[0] << " further from Link "
                  << result_vector[0].link_names[1] << "  : "
                  << result_vector[0].normal;
    }
    else {
        std::cout << "Collision" << std::endl;
        std::cout << result_vector.size() << " collision info" << std::endl;
        std::cout << "Distance: " << result_vector[0].distance << std::endl;

        std::cout << "Link: " << result_vector[0].link_names[0]
                  << " nearest point: " << result_vector[0].nearest_points[0]
                  << std::endl;
        std::cout << "Link: " << result_vector[0].link_names[1]
                  << " nearest point: " << result_vector[0].nearest_points[1]
                  << std::endl;
        std::cout << "Direction to move Link " << result_vector[0].link_names[1]
                  << " further from Link " << result_vector[0].link_names[0]
                  << "  : " << result_vector[0].normal;
    }

    return 0;
}