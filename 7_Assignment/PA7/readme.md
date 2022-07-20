question:
1.Sence::sampleLight()中无法处理多个光源的情况?

note:
1.BVH构建框架
实例化Scene场景
加载物体scene，对象是meshTriangles
在meshTriangles构建时，加载生成该物体的Triangles，并且构建bvh(该物体内部bvh)
调用scene方法BuildBvh(构建外部bvh)
2.光线和包围盒求交
scene的BVH只包含meshTriangle物体，可以理解为具体的实体，bvh比较小
当到叶子节点时，调用node->object->getIntersection()方法，调用实体内部的bvh求交，才是理解上的Triangle求交