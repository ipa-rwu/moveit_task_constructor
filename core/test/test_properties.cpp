#include <moveit/task_constructor/properties.h>

#include <gtest/gtest.h>
#include <initializer_list>

using namespace moveit::task_constructor;

TEST(Property, standard) {
	PropertyMap props;
	props.declare<double>("double1", 1, "first");
	props.declare<double>("double2", 2);
	props.declare<double>("double4");

	EXPECT_EQ(props.get<double>("double1"), 1.0);
	EXPECT_EQ(props.get<double>("double2"), 2.0);
	EXPECT_THROW(props.get<double>("double3"), std::runtime_error);

	EXPECT_THROW(props.get<double>("double4"), std::runtime_error);
	EXPECT_FALSE(props.property("double4").defined());
	EXPECT_EQ(props.get<double>("double4", 0.0), 0.0);

	props.set("double3", 3.0);
	EXPECT_EQ(props.get<double>("double3"), 3.0);
}

TEST(Property, redeclare) {
	PropertyMap props;
	props.declare<double>("double1");

	// avoid second declaration with different type
	props.declare<double>("double1");
	EXPECT_THROW(props.declare<long double>("double1"), std::logic_error);

	// types not matching?
	EXPECT_THROW(props.set("double1", 1), std::logic_error);

	props.set("double1", 3.14);
	EXPECT_EQ(props.get<double>("double1"), 3.14);
}

TEST(Property, reset) {
	PropertyMap props;
	props.declare<double>("double1");

	// setCurrent() only assigns temporary values
	props.setCurrent("double1", 1.0);
	ASSERT_EQ(props.get<double>("double1"), 1.0);
	EXPECT_TRUE(props.property("double1").defaultValue().empty());

	// they can be reset to their defaults
	props.reset();
	EXPECT_FALSE(props.property("double1").defined());

	// set() also updates the default
	props.set("double1", 1.0);
	EXPECT_EQ(props.get<double>("double1"), 1.0);
	EXPECT_EQ(boost::any_cast<double>(props.property("double1").defaultValue()), 1.0);

	props.setCurrent("double1", 2.0);
	EXPECT_EQ(props.get<double>("double1"), 2.0);
	EXPECT_EQ(boost::any_cast<double>(props.property("double1").defaultValue()), 1.0);

	// back to default
	props.reset();
	EXPECT_EQ(props.get<double>("double1"), 1.0);
}

class InitFromTest : public ::testing::Test {
protected:
	void SetUp() {
		master.declare<double>("double1", 1);
		master.declare<double>("double2", 2);
		master.declare<double>("double4", 4);

		slave.declare<double>("double1");
		slave.declare<double>("double2");
		slave.declare<double>("double3");
	}
	PropertyMap master;
	PropertyMap slave;
};

TEST_F(InitFromTest, standard) {
	slave.configureInitFrom(0);  // init all matching vars
	ASSERT_FALSE(slave.property("double1").defined());

	slave.performInitFrom(0, master);
	EXPECT_EQ(slave.get<double>("double1"), 1.0);
	EXPECT_EQ(slave.get<double>("double2"), 2.0);
	EXPECT_FALSE(slave.property("double3").defined());
	EXPECT_THROW(slave.property("double4"), std::runtime_error);
}

TEST_F(InitFromTest, limited) {
	slave.configureInitFrom(0, {"double1"}); // limit init to listed props
	slave.performInitFrom(0, master);
	EXPECT_EQ(slave.get<double>("double1"), 1.0);
	EXPECT_FALSE(slave.property("double2").defined());
	EXPECT_FALSE(slave.property("double3").defined());
	EXPECT_THROW(slave.property("double4"), std::runtime_error);
}

TEST_F(InitFromTest, sourceId) {
	slave.configureInitFrom(0);  // init all matching vars
	slave.performInitFrom(1, master); // init with wrong sourceId -> no effect
	EXPECT_FALSE(slave.property("double1").defined());
	EXPECT_FALSE(slave.property("double2").defined());
	EXPECT_FALSE(slave.property("double3").defined());
	EXPECT_THROW(slave.property("double4"), std::runtime_error);
}

TEST_F(InitFromTest, otherName) {
	slave.property("double1").configureInitFrom(0, "double2"); // init double1 from double2
	slave.performInitFrom(0, master);
	EXPECT_EQ(slave.get<double>("double1"), 2.0);
	EXPECT_FALSE(slave.property("double2").defined());
	EXPECT_FALSE(slave.property("double3").defined());
	EXPECT_THROW(slave.property("double4"), std::runtime_error);
}

TEST_F(InitFromTest, function) {
	slave.property("double3").configureInitFrom(0, [](const PropertyMap& other) -> boost::any {
		return other.get<double>("double1") + other.get<double>("double2");
	});
	slave.performInitFrom(0, master);
	EXPECT_EQ(slave.get<double>("double3"), 3.0);
}