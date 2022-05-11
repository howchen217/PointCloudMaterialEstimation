//
// Created by haocheng on 8-5-22.
//

#include "PCPointMaterialDirector.h"



void PCPointMaterialDirector::setBuilder(PCPointMaterialBuilder* builder) {
    pcPointMaterialBuilder = builder;
}

void PCPointMaterialDirector::buildPCPointMaterial() {
    pcPointMaterialBuilder->buildBasicData();
    pcPointMaterialBuilder->buildLaserDirection();
    pcPointMaterialBuilder->buildNormalDirection();
    pcPointMaterialBuilder->buildIncidenceAngle();
    pcPointMaterialBuilder->buildCorrectedIntensity();
    pcPointMaterialBuilder->buildReflectance();
    pcPointMaterialBuilder->buildAlbedo();
    pcPointMaterialBuilder->buildEmissivity();
}

PCPointMaterialDirector::PCPointMaterialDirector() {

}
