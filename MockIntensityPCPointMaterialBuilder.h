//
// Created by haocheng on 19-5-22.
//

#ifndef PCMATTEX_MOCKINTENSITYPCPOINTMATERIALBUILDER_H
#define PCMATTEX_MOCKINTENSITYPCPOINTMATERIALBUILDER_H


#include "Vector3.hpp"
#include "PCPointMaterial.h"
#include "PCPointMaterialBuilder.h"

/**
 * \brief Child builder of PCPointMaterialBuilder, use mock intensity and does intensity correction.
 *
 * Use grayscale as mock intensity and still performs intensity correction.
 */
class MockIntensityPCPointMaterialBuilder : public PCPointMaterialBuilder {
private:

    Vector3 rgb;
    Vector3 point_coordinate;
    Vector3 scanner_position;
    Vector3 point_normal;

    PCPointMaterial pcPointMaterial;


public:

    /**
     * \brief Default constructor of MockIntensityPCPointMaterialBuilder.
     */
    MockIntensityPCPointMaterialBuilder();

    /**
     * \brief Constructor of MockIntensityPCPointMaterialBuilder.
     *
     * It does not require input of raw intensity as intensity is mocked by grayscale.
     *
     * @param rgb The RGB information of the point cloud point.
     * @param point_coordinate The point coordinate of the point cloud point.
     * @param scanner_position The scanner position of the point cloud point.
     * @param point_normal The normal of the point cloud point.
     */
    MockIntensityPCPointMaterialBuilder(const Vector3 &rgb, const Vector3 &point_coordinate,
                                        const Vector3 &scanner_position, const Vector3 &point_normal);

    /**
     * \brief Set the input values and calculate mock intensity.
     *
     * Set rgb, point coordinate, scanner position and point normal. Calculate and set the mock intensity as grayscale
     * acquired from the rgb information.
     */
    void buildBasicData() override;

    /**
     * @copydoc PCPointMaterialBuilder::buildLaserDirection()
     */
    void buildLaserDirection() override;

    /**
     * @copydoc PCPointMaterialBuilder::buildNormalDirection()
     */
    void buildNormalDirection() override;

    /**
     * @copydoc PCPointMaterialBuilder::buildIncidenceAngle()
     */
    void buildIncidenceAngle() override;

    /**
     * @copydoc PCPointMaterialBuilder::buildCorrectedIntensity()
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


#endif //PCMATTEX_MOCKINTENSITYPCPOINTMATERIALBUILDER_H
