//
// Created by haocheng on 8-5-22.
//

#ifndef PCMATTEX_PCPOINTMATERIALDIRECTOR_H
#define PCMATTEX_PCPOINTMATERIALDIRECTOR_H

#include "PCPointMaterialBuilder.h"

class PCPointMaterialDirector {
private:
    PCPointMaterialBuilder* pcPointMaterialBuilder;

public:
    PCPointMaterialDirector();

    void setBuilder(PCPointMaterialBuilder* builder);
    void buildPCPointMaterial();
};


#endif //PCMATTEX_PCPOINTMATERIALDIRECTOR_H
