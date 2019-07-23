
#include "appShowKinectData.h"

#include "visualize/showKinectData.h"
#include "visualize/showRGBDData.h"
#include "visualize/showRegisterTwoRigideFrames.h"
#include "visualize/showSensData.h"
#include "visualize/showMesh.h"

using namespace ml;


void AppShowKinectData::init(ml::ApplicationData &app)
{
	last_time_key_was_pressed = std::chrono::system_clock::now();
	m_render_data = std::make_unique<ShowMesh>();
	//m_render_data = std::make_unique<ShowRGBDImageData>();
	//m_render_data = std::make_unique<ShowTwoRigideRegisteredFrames>();
	//m_render_data = std::make_unique<ShowSensData>();
	//m_render_data = std::make_unique<ShowKinectData>();
	m_render_data->init(app);

	ml::vec3f eye(-0.5f, -0.5f, 1.5f);
	m_camera = Cameraf(eye, vec3f::eY, vec3f::eZ, 60.0f, (float)app.window.getWidth() / app.window.getHeight(), 0.01f, 10.0f);

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

	auto time_passed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - last_time_key_was_pressed).count();
	if (last_pressed_key != key || time_passed > 500) {
		m_render_data->key(key);
		last_pressed_key = key;
		last_time_key_was_pressed = std::chrono::system_clock::now();
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
