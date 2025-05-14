#include <CustomInteractorStyle.h>

vtkStandardNewMacro(CustomInteractorStyle);

void CustomInteractorStyle::OnKeyPress()
{
    std::string key = this->Interactor->GetKeySym();

    if (key == "3")
    {
        return;
    }

    this->vtkInteractorStyleTrackballCamera::OnKeyPress();
}
