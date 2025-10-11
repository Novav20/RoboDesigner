#include <gtest/gtest.h>
#include <glm/glm.hpp>

TEST(GLMIntegration, VectorAddition) {
	// Arrange
	const glm::vec3 vector_a(1.0f, 2.0f, 3.0f);
	const glm::vec3 vector_b(4.0f, 5.0f, 6.0f);
	const glm::vec3 expected_result(5.0f, 7.0f, 9.0f);
	
	// Act
	const glm::vec3 actual_result = vector_a + vector_b;
	
	// Assert
	EXPECT_NEAR(actual_result.x, expected_result.x, 1e-9);
	EXPECT_NEAR(actual_result.y, expected_result.y, 1e-9);
	EXPECT_NEAR(actual_result.z, expected_result.z, 1e-9);
}