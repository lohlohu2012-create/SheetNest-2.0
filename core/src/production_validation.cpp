    };

    // Stage 0: adaptive destroy-and-repair. Only the conflict-driven local
    // group is extracted; unrelated placements remain fixed. Each round is
    // immediately checked by the Production Validator, and a new conflict
    // group is derived from any remaining issues.
    if (options.enableAdaptiveDestroyRepair &&
        !timeExpired()) {
        Result adaptiveCandidate = result;
        ProductionValidationReport adaptiveReport = initial;
        std::size_t adaptiveNeighborBudget =
            std::max<std::size_t>(
                1,
                options.adaptiveRepairMaxNeighbors
            );
        std::size_t previousIssueCount =
            adaptiveReport.issues.size();

        for (std::size_t round = 0;
             round < std::max<std::size_t>(
                 1,
                 options.adaptiveRepairRounds
             ) &&
             !timeExpired();
             ++round) {
            if (repairOptions.control &&
                repairOptions.control->shouldStop()) {
                break;
            }

            const auto seedIds =
                seedIdsFromReport(
                    adaptiveReport,
                    adaptiveCandidate,
                    instances,
                    repairOptions.gapMm,
                    adaptiveNeighborBudget,
                    round
                );

            if (seedIds.empty()) {
                break;
            }

            if (round == 0) {
                adaptiveConflictIds = seedIds;
            }

            for (const auto& id : seedIds) {
                adaptiveExtractedSet.insert(id);
            }

            largestAdaptiveGroup =
                std::max(
                    largestAdaptiveGroup,
                    seedIds.size()
                );

            const Result roundBefore =
                adaptiveCandidate;

            Result localCandidate = adaptiveCandidate;
            std::vector<std::string> roundExtractedIds;

            if (!adaptiveDestroyAndRepairResult(
                    instances,
                    sheet,
                    repairOptions,
                    seedIds,
                    localCandidate,
                    &roundExtractedIds
                )) {
                break;
            }

            ++actualAdaptiveRounds;

            for (const auto& id : roundExtractedIds) {
                adaptiveExtractedSet.insert(id);
            }

            adaptiveHistory.push_back(
                makeRoundSnapshot(
                    round + 1,
                    seedIds,
                    roundExtractedIds,
                    roundBefore,
                    localCandidate
                )
            );

            adaptiveReport =
                validateProductionResult(
                    instances,
                    sheet,
                    repairOptions,
                    localCandidate
                );

            // React to the actual repair result. If the number of validation
            // issues falls, keep the neighborhood focused. If it stagnates
            // or grows, widen the next destroy group. This avoids both
            // repeatedly repairing the same tiny group and unnecessarily
            // extracting a large portion of a good nest.
            const currentIssueCount =
                adaptiveReport.issues.size();

            if (currentIssueCount < previousIssueCount) {
                adaptiveNeighborBudget =
                    std::max<std::size_t>(
                        1,
                        (adaptiveNeighborBudget + 1) / 2
                    );
            } else if (currentIssueCount >= previousIssueCount) {
                adaptiveNeighborBudget =
                    std::min<std::size_t>(
                        std::max<std::size_t>(
                            1,
                            options.adaptiveRepairMaxNeighbors
                        ),
                        std::max<std::size_t>(
                            adaptiveNeighborBudget + 1,
                            adaptiveNeighborBudget * 2
                        )
                    );
            }

            previousIssueCount = currentIssueCount;

            if (adaptiveReport.valid) {
                if (!foundValid ||
                    validScore(localCandidate) <
                        validScore(bestValid)) {
                    bestValid =
                        localCandidate;
                    foundValid = true;
                    selectedAdaptiveCandidate = true;
                }
                break;
            }

            if (failureScore(
                    adaptiveReport,
                    localCandidate
                ) <
                failureScore(
                    bestFailureReport,
                    bestFailure