
#include "appShowKinectData.h"

#include "visualize/showKinectData.h"
#include "visualize/showRGBDData.h"


//void backProjectGraphics(const DepthImage32& depth, const ColorImageR8G8B8A8& color, const Cameraf& camera) {
//	PointCloudf pc;
//	mat4f inv = camera.getProj().getInverse();
//	for (auto& d : depth) {
//		if (depth.isValidValue(d.value)) {
//			vec2f p = D3D11GraphicsDevice::pixelToNDC(vec2i((int)d.x, (int)d.y), depth.getWidth(), depth.getHeight());
//			vec3f v = (inv * vec3f((float)p.x, (float)p.y, d.value));
//			pc.m_points.push_back(v);
//			pc.m_colors.push_back(vec4f(color(d.x, d.y)) / 255.0f);
//		}
//	}
//	PointCloudIOf::saveToFile("test_backproj_graphics.ply", pc);
//}
//
//void backProjectVision(const DepthImage32& _depth, const ColorImageR8G8B8A8& color, const Cameraf& camera) {
//	DepthImage32 depth = _depth;
//
//	{
//		//first get it to world space z
//		mat4f inv = camera.getProj().getInverse();
//		for (auto& d : depth) {
//			if (depth.isValidValue(d.value)) {
//				vec2f p = D3D11GraphicsDevice::pixelToNDC(vec2i((int)d.x, (int)d.y), depth.getWidth(), depth.getHeight());
//				vec3f v = (inv * vec3f((float)p.x, (float)p.y, d.value));
//				d.value = v.z;
//			}
//			else {
//				d.value = 0.0f;
//			}
//		}
//		depth.setInvalidValue(0.0f);
//	}
//
//	PointCloudf pc;
//	mat4f inv = camera.getIntrinsic(depth.getWidth(), depth.getHeight()).getInverse();
//	for (auto& d : depth) {
//		if (depth.isValidValue(d.value)) {
//			vec3f p((float)d.x * d.value, (float)d.y * d.value, d.value);
//			p = inv * p;
//
//			pc.m_points.push_back(p);
//			pc.m_colors.push_back(vec4f(color(d.x, d.y)) / 255.0f);
//		}
//	}
//	PointCloudIOf::saveToFile("test_backproj_vision.ply", pc);
//}
//

void AppShowKinectData::init(ml::ApplicationData &app)
{
	m_render_data = std::make_unique<ShowRGBDImageData>();
	m_render_data->init(app);

	ml::vec3f eye(-0.5f, -0.5f, 1.5f);
	m_camera = Cameraf(eye, -vec3f::eY, vec3f::eZ, 60.0f, (float)app.window.getWidth() / app.window.getHeight(), 0.01f, 10.0f);

	m_font.init(app.graphics, "Calibri");
	m_canvas.init(app.graphics);


	D3D11Canvas2D canvas = m_canvas;

	std::cout << "\nInit done!\n\n" << std::endl;
}

void AppShowKinectData::render(ml::ApplicationData& app)
{
	m_timer.frame();
	m_canvas.render();

	//mat4f proj = Cameraf::visionToGraphicsProj(app.window.getWidth(), app.window.getHeight(), 1108.51f, 1108.51f, m_camera.getNearPlane(), m_camera.getFarPlane());

	m_render_data->render(m_camera);

	m_font.drawString("FPS: " + ml::convert::toString(m_timer.framesPerSecond()), ml::vec2i(10, 5), 24.0f, ml::RGBColor::Red);
}

void AppShowKinectData::resize(ml::ApplicationData &app)
{
	m_canvas.resize();
	m_camera.updateAspectRatio((float)app.window.getWidth() / app.window.getHeight());
}

void AppShowKinectData::keyDown(ml::ApplicationData& app, UINT key)
{
	if (key == KEY_Z) {
		const unsigned int width = app.window.getWidth();
		const unsigned int height = app.window.getHeight();
		mat4f intrinsic = Cameraf::graphicsToVisionProj(m_camera.getProj(), width, height);
		mat4f extrinsic = m_camera.getExtrinsic();

		std::cout << "intrinsic before " << intrinsic << std::endl;
		std::cout << "extrinsic before " << extrinsic << std::endl;
		std::cout << "camera before: " << m_camera.toString() << std::endl;
		Cameraf newCam = Cameraf::visionToGraphics(extrinsic, width, height, intrinsic(0, 0), intrinsic(1, 1), m_camera.getNearPlane(), m_camera.getFarPlane());
		newCam.updateWorldUp(m_camera.getWorldUp());
		m_camera = newCam;

		intrinsic = Cameraf::graphicsToVisionProj(m_camera.getProj(), width, height);
		extrinsic = m_camera.getExtrinsic();
		std::cout << "camera after: " << m_camera.toString() << std::endl;
		std::cout << "intrinsic after " << intrinsic << std::endl;
		std::cout << "extrinsic after " << extrinsic << std::endl;
	}
}



void AppShowKinectData::keyPressed(ml::ApplicationData& app, UINT key)
{
	const float distance = 0.1f;
	const float theta = 5.0f;

	if (key == KEY_S) m_camera.move(-distance);
	if (key == KEY_W) m_camera.move(distance);
	if (key == KEY_A) m_camera.strafe(-distance);
	if (key == KEY_D) m_camera.strafe(distance);
	if (key == KEY_E) m_camera.jump(-distance);
	if (key == KEY_Q) m_camera.jump(distance);

	if (key == KEY_UP) m_camera.lookUp(theta);
	if (key == KEY_DOWN) m_camera.lookUp(-theta);
	if (key == KEY_LEFT) m_camera.lookRight(theta);
	if (key == KEY_RIGHT) m_camera.lookRight(-theta);

	if (key == KEY_F1) {
		auto color = app.graphics.castD3D11().captureBackBufferColor();
		auto depth = app.graphics.castD3D11().captureBackBufferDepth();
		FreeImageWrapper::saveImage("screenshot_color.png", color);
		FreeImageWrapper::saveImage("screenshot_depth.png", ColorImageR32G32B32A32(depth));
	}


	if (key == KEY_C) {
		std::cout << "proj:\n";
		std::cout << m_camera.getProj() << std::endl;
		std::cout << "view:\n";
		std::cout << m_camera.getView() << std::endl;
		std::cout
			<< m_camera.getEye() << "\n"
			<< m_camera.getRight() << "\n"
			<< m_camera.getUp() << "\n"
			<< m_camera.getLook() << "\n" << std::endl;
	}

	if (key == KEY_P) {
		auto color = app.graphics.castD3D11().captureBackBufferColor();
		auto depth = app.graphics.castD3D11().captureBackBufferDepth();
		//backProjectGraphics(depth, color, m_camera);
		//backProjectVision(depth, color, m_camera);
	}

	if (key == KEY_F2) {
		ml::D3D11RenderTarget renderTarget;
		renderTarget.init(app.graphics.castD3D11(), app.window.getWidth(), app.window.getHeight(), std::vector < DXGI_FORMAT > {DXGI_FORMAT_R8G8B8A8_UNORM}, true);
		renderTarget.clear();
		renderTarget.bind();
		render(app);
		renderTarget.unbind();
		ml::ColorImageR8G8B8A8 color;
		ml::DepthImage32 depth;
		renderTarget.captureColorBuffer(color);
		renderTarget.captureDepthBuffer(depth);
		FreeImageWrapper::saveImage("color.png", color);
		FreeImageWrapper::saveImage("depth.png", ml::ColorImageR32G32B32A32(depth));
	}

	if (key == 'R') {
		mat4f intrinsics = m_camera.getIntrinsic(app.window.getWidth(), app.window.getHeight());

		//std::cout << intrinsics << std::endl;
		mat4f intrinsicsInverse = intrinsics.getInverse();

		ml::mat4f projToCam = m_camera.getProj().getInverse();
		ml::mat4f camToWorld = m_camera.getView().getInverse();
		ml::ColorImageR32G32B32 image(app.window.getWidth(), app.window.getHeight());

		const std::string testFilename = "scans/gates381.ply";
		ml::MeshDataf meshData = ml::MeshIOf::loadFromFile(testFilename);
		ml::TriMeshf triMesh(meshData);

		ml::Timer c0;
		c0.start();
		ml::TriMeshAcceleratorBVHf accel(triMesh, false);
		std::cout << "time construct " << c0.getElapsedTimeMS() << std::endl;

		std::vector<const TriMeshRayAcceleratorf*> accelVec;
		accelVec.push_back(&accel);
		int s = sizeof(accel);

		ml::Timer c;
		c.start();
#pragma omp parallel for
		for (int y_ = 0; y_ < (int)app.window.getHeight(); y_++) {
			unsigned int y = (unsigned int)y_;

			for (unsigned int x = 0; x < app.window.getWidth(); x++) {

				float depth0 = 0.5f;
				float depth1 = 1.0f;
				vec4f p0 = camToWorld * intrinsicsInverse*vec4f((float)x*depth0, (float)(y)*depth0, depth0, 1.0f);
				vec4f p1 = camToWorld * intrinsicsInverse*vec4f((float)x*depth1, (float)(y)*depth1, depth1, 1.0f);

				vec3f eye = m_camera.getEye();
				Rayf r(m_camera.getEye(), (p1.getVec3() - p0.getVec3()).getNormalized());

				Rayf _check = m_camera.getScreenRay((float)x / app.window.getWidth(), (float)y / app.window.getHeight());

				int a = 5;
				float t, u, v;
				const ml::TriMeshf::Triangle* tri;
				unsigned int objIdx;
				TriMeshRayAcceleratorf::Intersection intersect = TriMeshRayAcceleratorf::getFirstIntersection(r, accelVec, objIdx);
				t = intersect.t;
				u = intersect.u;
				v = intersect.v;
				tri = intersect.triangle;

				if (tri) {
					image(x, y) = tri->getSurfaceColor(u, v).getVec3();
				}
				else {
					image(x, y) = 0;
				}

			}
		}
		double elapsed = c.getElapsedTimeMS();
		std::cout << "time trace " << elapsed << std::endl;
		unsigned int raysPerSec = (unsigned int)((double)(app.window.getHeight()*app.window.getWidth()) / (elapsed / 1000.0));
		std::cout << "million rays/s " << (float)raysPerSec / 1000000.0 << std::endl;

		ml::FreeImageWrapper::saveImage("test.jpg", image);
	}
}

void AppShowKinectData::mouseDown(ml::ApplicationData &app, ml::MouseButtonType button)
{

}

void AppShowKinectData::mouseWheel(ml::ApplicationData &app, int wheelDelta)
{
	const float distance = 0.002f;
	m_camera.move(distance * wheelDelta);
}

void AppShowKinectData::mouseMove(ml::ApplicationData &app)
{
	const float distance = 0.01f;
	const float theta = 0.5f;

	ml::vec2i posDelta = app.input.mouse.pos - app.input.prevMouse.pos;

	if (app.input.mouse.buttons[ml::MouseButtonRight])
	{
		m_camera.strafe(distance * posDelta.x);
		m_camera.jump(distance * posDelta.y);
	}

	if (app.input.mouse.buttons[ml::MouseButtonLeft])
	{
		m_camera.lookRight(theta * posDelta.x);
		m_camera.lookUp(theta * posDelta.y);
	}

}
