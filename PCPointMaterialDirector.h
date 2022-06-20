//
// Created by haocheng on 8-5-22.
//

#ifndef PCMATTEX_PCPOINTMATERIALDIRECTOR_H
#define PCMATTEX_PCPOINTMATERIALDIRECTOR_H

#include "PCPointMaterialBuilder.h"

/**
 * \brief Director for building PCPointMaterial.
 *
 * Follows the builder design pattern.
 */
class PCPointMaterialDirector {
private:
    PCPointMaterialBuilder* pcPointMaterialBuilder;

public:
    /**
     * \brief Constructor of PCPointMaterialDirector.
     */
    PCPointMaterialDirector();

    /**
     * Set the PCPointMaterial builder used by the director.
     *
     * @param builder The builder.
     */
    void setBuilder(PCPointMaterialBuilder *builder);

    /**
     * \brief Build the PCPointMaterial.
     */
    void buildPCPointMaterial();
};


#endif //PCMATTEX_PCPOINTMATERIALDIRECTOR_H
