        assert(placement.x + 50.0 <= 80.0 + 1e-6);
        assert(placement.y + 50.0 <= 80.0 + 1e-6);
    }
    assert(foundInsert);
}

} // namespace

int main() {
    testGeometry();
    testDxfHoleRecovery();
    testLayerSeparation();
    testLineArcClosure();
    testBulgePolyline();
    testLegacyPolyline();
    testMultipleParts();
    testOpenPolylineJoining();
    testDegenerateArc();
    testDxfModelPipeline();
    testPerPartQuantitiesAndUnitIds();
    testDxfExportRoundTrip();
    testCollinearConcaveNfpRegression();
    testClearanceCornerSampling();
    testNfpMinkowski();
    testContinuousConcaveFeasibilityRegion();
    testFeasibilitySamplingBudget();
    testFeasibilityGap();
    testNfpUnionAndCache();
    testConcaveUnionNfp();
    testConcaveNfpCandidates();
    testReadableValidationErrors();
    testMinimumSheets();
    testProductionValidator();
    testSpatialIndexBroadPhase();
    testAdaptiveDestroyAndRepair();
    testAutomaticProductionRepair();
}
