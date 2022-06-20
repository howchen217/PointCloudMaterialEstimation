//
// Created by haocheng on 7-5-22.
//

#ifndef PCMATTEX_PCPOINTMATERIALBUILDER_H
#define PCMATTEX_PCPOINTMATERIALBUILDER_H

#include "Vector3.hpp"
#include "PCPointMaterial.h"

/**
 * The builder class for PCPointMaterial, following the builder design pattern.
 * It keeps the calculation in the correct order to avoid skipping preliminary results.
 */
class PCPointMaterialBuilder {
private:
    float raw_intensity;
    Vector3 rgb;
    Vector3 point_coordinate;
    Vector3 point_normal;
    Vector3 scanner_position;

    PCPointMaterial pcPointMaterial;


public:

    /**
     * \brief The default constructor of PCPointMaterialBuilder.
     */
    PCPointMaterialBuilder();

    /**
     * \brief Constructor of PCPointMaterial Builder.
     *
     * This builder includes point intensity as well as normal direction. It expects a point cloud with intensity.
     *
     * @param raw_intensity The raw intensity of the point cloud point.
     * @param rgb The rgb of the point cloud point.
     * @param point_coordinate The point coordinate of the point cloud point.
     * @param point_normal The point normal of the point cloud point.
     * @param scanner_position The scanner position of the point cloud point.
     */
    PCPointMaterialBuilder(float raw_intensity, const Vector3 &rgb, const Vector3 &point_coordinate,
                           const Vector3 &point_normal, const Vector3 &scanner_position);


    /**
     * \brief Set the input values of raw intensity, rgb, point coordinate and scanner position.
     *
     * This is the first step.
     */
    virtual void buildBasicData();

    /**
     * \brief Calculate the laser direction.
     *
     * This is the second step.
     */
    virtual void buildLaserDirection();

    /**
     * \brief Set the normal direction.
     *
     * This is the third step.
     */
    virtual void buildNormalDirection();

    /**
     * \brief Calculates incidence angle of the laser hitting the point cloud point.
     *
     * This is the fourth step.
     */
    virtual void buildIncidenceAngle();

    /**
     * \brief Calculate the corrected intensity.
     *
     * This is the fifth step.
     */
    virtual void buildCorrectedIntensity();

    /**
     * \brief Calculate the reflectance of the material.
     *
     * The material property calculation can be done in parallel, since preliminary results are already acquired.
     */
    virtual void buildReflectance();

    /**
     * \brief Calculate the albedo of the material.
     *
     * The material property calculation can be done in parallel, since preliminary results are already acquired.
     */
    virtual void buildAlbedo();

    /**
     * \brief Calculate the emissivity of the material.
     *
     * The material property calculation can be done in parallel, since preliminary results are already acquired.
     */
    virtual void buildEmissivity();

    /**
     * \brief Get the PCPointMaterial, should be done after the PCPointMaterialDirector builds.
     *
     * @return The PCPointMaterial.
     */
    virtual PCPointMaterial getPCPointMaterial();



};


#endif //PCMATTEX_PCPOINTMATERIALBUILDER_H
