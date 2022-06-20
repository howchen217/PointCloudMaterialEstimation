//
// Created by haocheng on 19-5-22.
//

#ifndef PCMATTEX_MOCKINTENSITYNOCORRECTIONPCPOINTMATERIALBUILDER_H
#define PCMATTEX_MOCKINTENSITYNOCORRECTIONPCPOINTMATERIALBUILDER_H


#include "Vector3.hpp"
#include "PCPointMaterial.h"
#include "PCPointMaterialBuilder.h"

/**
 * \brief Child builder of PCPointMaterialBuilder, it mock intensity and does not do intensity correction.
 *
 * Use grayscale to mock intensity and skip the entire step of intensity correction.
 */
class MockIntensityNoCorrectionPCPointMaterialBuilder : public PCPointMaterialBuilder{
private:
    Vector3 rgb;
    Vector3 point_coordinate;

    PCPointMaterial pcPointMaterial;


public:

    /**
     * \brief The default constructor of MockIntensityNoCorrectionMaterialBuilder.
     */
    MockIntensityNoCorrectionPCPointMaterialBuilder();

    /**
     * \brief The constructor of MockIntensityNoCorrectionPCPointMaterialBuilder.
     * @param rgb The RGB information.
     * @param point_coordinate The point coordinate.
     */
    MockIntensityNoCorrectionPCPointMaterialBuilder(const Vector3 &rgb, const Vector3 &point_coordinate);

    /**
     * \brief Set the input values, only rgb and point coordinate is needed.
     */
    void buildBasicData() override;

    /**
     * @copydoc PCPointMaterialBuilder::buildLaserDirection()
     *
     * This step is skipped since the step of intensity correction is skipped.
     */
    void buildLaserDirection() override;

    /**
     * @copydoc PCPointMaterialBuilder::buildNormalDirection()
     *
     * This step is skipped since the step of intensity correction is skipped.
     */
    void buildNormalDirection() override;

    /**
     * @copydoc PCPointMaterialBuilder::buildIncidenceAngle()
     *
     * This step is skipped since the step of intensity correction is skipped.
     */
    void buildIncidenceAngle() override;

    /**
     * @copydoc PCPointMaterialBuilder::buildCorrectedIntensity()
     *
     * This step is skipped since the step of intensity correction is skipped.
     */
    void buildCorrectedIntensity() override;

    /**
     * @copydoc PCPointMaterialBuilder::buildReflectance()
     */
    void buildReflectance() override;

    /**
     * @copydoc PCPointMaterialBuilder::buildAlbedo()
     */
    void buildAlbedo() override;

    /**
     * @copydoc PCPointMaterialBuilder::buildEmissivity()
     */
    void buildEmissivity() override;

    /**
     * @copydoc PCPointMaterialBuilder::getPCPointMaterial()
     */
    PCPointMaterial getPCPointMaterial() override;


};


#endif //PCMATTEX_MOCKINTENSITYNOCORRECTIONPCPOINTMATERIALBUILDER_H
