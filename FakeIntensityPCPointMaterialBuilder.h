//
// Created by haocheng on 19-5-22.
//

#ifndef PCMATTEX_FAKEINTENSITYPCPOINTMATERIALBUILDER_H
#define PCMATTEX_FAKEINTENSITYPCPOINTMATERIALBUILDER_H


#include "Vector3.hpp"
#include "PCPointMaterial.h"
#include "PCPointMaterialBuilder.h"

class FakeIntensityPCPointMaterialBuilder : public PCPointMaterialBuilder {
private:

    Vector3 rgb;
    Vector3 point_coordinate;
    Vector3 scanner_position;
    Vector3 point_normal;

    PCPointMaterial pcPointMaterial;


public:
    FakeIntensityPCPointMaterialBuilder();

    FakeIntensityPCPointMaterialBuilder(const Vector3 &rgb, const Vector3 &point_coordinate,
                                        const Vector3 &scanner_position, const Vector3 &point_normal);


    void buildBasicData() override;
    void buildLaserDirection() override;
    void buildNormalDirection() override;
    void buildIncidenceAngle() override;
    void buildCorrectedIntensity() override;
    void buildReflectance() override;
    void buildAlbedo() override;
    void buildEmissivity() override;

    PCPointMaterial getPCPointMaterial() override;

};


#endif //PCMATTEX_FAKEINTENSITYPCPOINTMATERIALBUILDER_H
