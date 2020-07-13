#include "DesignViewer.h"
#include "DesignViewerCommon.h"
#include <fstream>
#include <random>

int main(int argc, char* argv[]) {
    design_viewer::DesignViewer viewer;
    
    viewer.Initialize(design_viewer::k_param_viewer_fps);
    viewer.Run();
    viewer.Cleanup();
    
    return 0;
}
