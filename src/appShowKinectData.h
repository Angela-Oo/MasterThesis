#pragma once
#include "mLibInclude.h"
#include "visualize/i_showData.h"

class AppShowKinectData : public ml::ApplicationCallback
{
public:
	void init(ml::ApplicationData &app) override;
	void render(ml::ApplicationData &app) override;
	void keyDown(ml::ApplicationData &app, UINT key) override;
	void keyPressed(ml::ApplicationData &app, UINT key) override;
	void mouseDown(ml::ApplicationData &app, ml::MouseButtonType button) override;
	void mouseMove(ml::ApplicationData &app) override;
	void mouseWheel(ml::ApplicationData &app, int wheelDelta) override;
	void resize(ml::ApplicationData &app) override;
private:
	std::unique_ptr<IShowData> m_render_data;

	ml::D3D11Font m_font;
	ml::FrameTimer m_timer;

	ml::D3D11Canvas2D m_canvas;
	ml::Cameraf m_camera;
};