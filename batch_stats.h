#ifndef BATCH_STATS_H
#define BATCH_STATS_H

// ============================================================================
//  任务 1、2：批量蒙特卡洛模拟 + CSV 统计（凸多边形版本）
//  ---------------------------------------------------------------------------
//  用法（不打开图形窗口）：
//      rollingstone.exe --batch 200 2026
//      rollingstone.exe --batch 500 0      // 种子 0 表示用当前时间
//
//  每次模拟随机抽样（变异系数 CV = 0.1）：
//      x0   ~ U(90, 100) m              释放点（坡脚上方 30° 陡坡段）
//      v0   ~ N(10.0, 1.0) m/s         顺坡初速度（模拟已经加速的滚石）
//      r    ~ N(2.0, 0.20) m            外接圆半径
//      S    ~ N(3.0, 0.30) m^2          截面积，夹到 [1e-4 r^2, 0.95*maxArea(3,r)]
//      mu   ~ N(0.45, 0.045)            摩擦系数
//      e    ~ N(0.45, 0.045)            恢复系数
//      Cd   ~ N(1.0, 0.10)              空气阻力系数
//      c_rr ~ N(0.02, 0.002)            滚动阻力系数
//      边数 k 在 [3, 6] 内均匀随机，顶点由 generatePolygon 的定面积采样生成。
//
//  坡脚分析区：地面高程 z <= 10 m（x 从 10 m 等高线到坡脚），1 m x 1 m 网格。
//      p_all     = 经过该格的模拟次数 / 总次数
//      p_entered = 经过该格的模拟次数 / 进入坡脚区的次数
//
//  输出（当前工作目录）：
//      batch_trials.csv, route_probability_grid.csv, route_profile_x.csv,
//      impact_height_at_toe.csv, trajectories_toe.csv, batch_summary.csv
//
//  说明：本头文件必须在 HillSlope / PolygonShape / RollingStone 定义之后、
//        main 之前包含；它复用主程序的物理模型和全局参数。
// ============================================================================

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <random>
#include <stdexcept>
#include <vector>

static double batchClamp(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static double batchMean(double sum, int n) {
    return (n > 0) ? sum / (double)n : 0.0;
}

static double batchStd(double sum, double sum2, int n) {
    if (n <= 1) return 0.0;
    double mean = sum / (double)n;
    double var = (sum2 - (double)n * mean * mean) / (double)(n - 1);
    if (var < 0.0) var = 0.0;
    return sqrt(var);
}

static void batchCsvText(char* dst, size_t cap, const char* src) {
    if (!dst || cap == 0) return;
    size_t j = 0;
    for (size_t i = 0; src && src[i] && j + 1 < cap; ++i) {
        char c = src[i];
        if (c == ',' || c == '"' || c == '\n' || c == '\r') c = ' ';
        dst[j++] = c;
    }
    dst[j] = 0;
}

static int runBatchSimulation(int nTrials, unsigned int seed) {
    if (nTrials < 1) nTrials = 1;
    if (nTrials > 20000) nTrials = 20000;
    if (seed == 0) seed = (unsigned int)std::time(nullptr);

    const double TOE_HEIGHT = 10.0;
    const double CELL_X = 1.0;
    const double CELL_Z = 1.0;
    const double MAX_BATCH_TIME = 30.0;
    const double TRAJ_SAMPLE_DT = 0.05;

    HillSlope hill;
    const double endX = (double)hill.getEndX();
    const double collectorX = endX - 10.0;   // 坡脚平台上的虚拟收集器 x=110 m

    double toeStartX = endX;
    for (double x = (double)hill.getStartX(); x <= endX; x += 0.05) {
        if (hill.getHeight(x) <= TOE_HEIGHT + 1e-9) { toeStartX = x; break; }
    }

    double toeWidth = endX - toeStartX;
    if (toeWidth < CELL_X) toeWidth = CELL_X;
    int nCols = (int)ceil(toeWidth / CELL_X - 1e-9);
    int nRows = (int)ceil(TOE_HEIGHT / CELL_Z - 1e-9);
    if (nCols < 1) nCols = 1;
    if (nRows < 1) nRows = 1;
    int nCells = nCols * nRows;

    std::vector<int> cellHits(nCells, 0), cellStamp(nCells, -1);
    std::vector<int> colHits(nCols, 0), colStamp(nCols, -1);
    std::vector<int> impactHits(nRows, 0);

    FILE* fpTrials  = std::fopen("batch_trials.csv", "w");
    FILE* fpGrid    = std::fopen("route_probability_grid.csv", "w");
    FILE* fpProfile = std::fopen("route_profile_x.csv", "w");
    FILE* fpImpact  = std::fopen("impact_height_at_toe.csv", "w");
    FILE* fpTraj    = std::fopen("trajectories_toe.csv", "w");
    FILE* fpSummary = std::fopen("batch_summary.csv", "w");

    if (!fpTrials || !fpGrid || !fpProfile || !fpImpact || !fpTraj || !fpSummary) {
        std::printf("ERROR: cannot create CSV files in the current directory.\n");
        if (fpTrials)  std::fclose(fpTrials);
        if (fpGrid)    std::fclose(fpGrid);
        if (fpProfile) std::fclose(fpProfile);
        if (fpImpact)  std::fclose(fpImpact);
        if (fpTraj)    std::fclose(fpTraj);
        if (fpSummary) std::fclose(fpSummary);
        return 1;
    }

    std::fprintf(fpTrials,
        "trial,x0,ground_z0,v0,radius,area,sides,mu,e,cd,crr,"
        "final_x,final_z,final_speed,max_speed,max_height,sim_time,"
        "entered_toe,reached_collector,collector_time,collector_z,"
        "collected_in_10m,reached_end,stopped,timed_out,failed,error,"
        "path_length,toe_entry_time\n");
    std::fprintf(fpGrid,
        "x_min,x_max,z_min,z_max,center_x,center_z,ground_z,"
        "hit_trials,p_all,p_entered\n");
    std::fprintf(fpProfile,
        "x_min,x_max,center_x,ground_z,hit_trials,p_all,p_entered\n");
    std::fprintf(fpImpact,
        "z_min,z_max,center_z,hit_trials,p_all,p_collected_0_10m\n");
    std::fprintf(fpTraj, "trial,time,x,z,speed\n");

    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> ux(90.0, 100.0);
    std::normal_distribution<double> nV0(10.0, 1.0);
    std::normal_distribution<double> nR(2.0, 0.20);
    std::normal_distribution<double> nArea(3.0, 0.30);
    std::normal_distribution<double> nMu(0.45, 0.045);
    std::normal_distribution<double> nE(0.45, 0.045);
    std::normal_distribution<double> nCd(1.0, 0.10);
    std::normal_distribution<double> nCrr(0.02, 0.002);

    int nEntered = 0, nCollector = 0, nCollected10 = 0;
    int nReachedEnd = 0, nStopped = 0, nTimeout = 0, nFailed = 0, nSuccess = 0;
    double sumFX = 0, sumFX2 = 0, sumFZ = 0, sumFZ2 = 0;
    double sumMaxV = 0, sumMaxV2 = 0, sumPath = 0, sumPath2 = 0;
    double sumTime = 0, sumTime2 = 0;

    std::printf("Running %d Monte-Carlo trials (seed=%u)...\n", nTrials, seed);
    std::printf("Toe region: x in [%.2f, %.2f] m, z in [0, %.1f] m\n",
                toeStartX, endX, TOE_HEIGHT);

    for (int trial = 0; trial < nTrials; ++trial) {
        double x0 = ux(rng);
        double groundZ0 = hill.getHeight(x0);
        double v0 = batchClamp(nV0(rng), 0.0, 20.0);
        double radius = batchClamp(nR(rng), 0.5, 3.0);
        double mu = batchClamp(nMu(rng), 0.05, 1.0);
        double e = batchClamp(nE(rng), 0.0, 0.9);
        double cd = batchClamp(nCd(rng), 0.0, 2.0);
        double crr = batchClamp(nCrr(rng), 0.0, 0.3);

        const int kMin = 3, kMax = 6;
        double maxArea = maxPolygonArea(kMin, radius);
        double areaLow = std::max(1.01e-4 * radius * radius, 0.01);
        double areaHigh = 0.95 * maxArea;
        if (areaHigh < areaLow) areaHigh = areaLow;
        double area = batchClamp(nArea(rng), areaLow, areaHigh);

        g_slideFriction = mu;
        g_restitution = e;
        g_dragCoef = cd;
        g_rollResist = crr;

        bool failed = false;
        char errorMsg[160] = "";
        int sides = 0;
        double finalX = x0, finalZ = groundZ0, finalSpeed = v0;
        double maxSpeed = v0, maxHeight = groundZ0;
        double t = 0.0, pathLength = 0.0, toeEntryTime = -1.0;
        double collectorTime = -1.0, collectorZ = 0.0;
        bool enteredToe = false, reachedCollector = false, collectedIn10m = false;
        bool reachedEnd = false, stopped = false, timedOut = false;
        int impactBin = -1;

        try {
            ShapeSettings ss{ kMin, kMax, radius, area };
            PolygonShape shape = generatePolygon(ss, rng);
            sides = (int)shape.local.size();
            area = shape.area;

            RollingStone stone(x0, groundZ0, shape);
            stone.adjustToGround(hill);
            stone.startAlongSlope(hill, v0);
            stone.setTrailMax(0);

            finalX = stone.getX();
            finalZ = stone.getZ();
            finalSpeed = stone.getSpeed();
            maxSpeed = finalSpeed;
            maxHeight = finalZ;

            double prevX = stone.getX();
            double prevZ = stone.getZ();
            double nextTrajTime = 0.0;

            while (t < MAX_BATCH_TIME && stone.isMoving()) {
                stone.update(hill, DT);
                t += DT;

                double x = stone.getX();
                double z = stone.getZ();
                double sp = stone.getSpeed();
                double dxs = x - prevX;
                double dzs = z - prevZ;
                pathLength += sqrt(dxs * dxs + dzs * dzs);
                if (sp > maxSpeed) maxSpeed = sp;
                if (z > maxHeight) maxHeight = z;

                if (!reachedCollector && prevX < collectorX && x >= collectorX) {
                    double denom = x - prevX;
                    double frac = (denom > 1e-12) ? (collectorX - prevX) / denom : 1.0;
                    if (frac < 0.0) frac = 0.0;
                    if (frac > 1.0) frac = 1.0;
                    collectorZ = prevZ + frac * (z - prevZ);
                    collectorTime = (t - DT) + frac * DT;
                    reachedCollector = true;
                    ++nCollector;
                    if (collectorZ >= 0.0 && collectorZ <= TOE_HEIGHT) {
                        collectedIn10m = true;
                        ++nCollected10;
                        impactBin = (int)(collectorZ / CELL_Z);
                        if (impactBin < 0) impactBin = 0;
                        if (impactBin >= nRows) impactBin = nRows - 1;
                        ++impactHits[impactBin];
                    }
                }

                bool inToe = (x >= toeStartX - 1e-9) && (x <= endX + 1e-9) &&
                             (z >= 0.0) && (z <= TOE_HEIGHT + 1e-9);
                if (inToe) {
                    if (!enteredToe) {
                        enteredToe = true;
                        toeEntryTime = t;
                        ++nEntered;
                    }
                    int col = (int)((x - toeStartX) / CELL_X);
                    int row = (int)(z / CELL_Z);
                    if (col < 0) col = 0;
                    if (col >= nCols) col = nCols - 1;
                    if (row < 0) row = 0;
                    if (row >= nRows) row = nRows - 1;
                    int idx = row * nCols + col;
                    if (cellStamp[idx] != trial) {
                        cellStamp[idx] = trial;
                        ++cellHits[idx];
                    }
                    if (colStamp[col] != trial) {
                        colStamp[col] = trial;
                        ++colHits[col];
                    }
                    if (t >= nextTrajTime) {
                        std::fprintf(fpTraj, "%d,%.4f,%.4f,%.4f,%.4f\n",
                                     trial, t, x, z, sp);
                        nextTrajTime = t + TRAJ_SAMPLE_DT;
                    }
                }

                prevX = x;
                prevZ = z;

                if (x >= endX - 0.5) { reachedEnd = true; break; }
            }

            finalX = stone.getX();
            finalZ = stone.getZ();
            finalSpeed = stone.getSpeed();
            stopped = (!stone.isMoving()) && (!reachedEnd);
            timedOut = (t >= MAX_BATCH_TIME - 1e-9) && stone.isMoving();
        }
        catch (const std::exception& ex) {
            failed = true;
            ++nFailed;
            batchCsvText(errorMsg, sizeof(errorMsg), ex.what());
        }

        if (failed) {
            // 失败的尝试不计入路线概率；撤销它在网格/收集器上的计数。
            for (int i = 0; i < nCells; ++i) {
                if (cellStamp[i] == trial) { --cellHits[i]; cellStamp[i] = -1; }
            }
            for (int c = 0; c < nCols; ++c) {
                if (colStamp[c] == trial) { --colHits[c]; colStamp[c] = -1; }
            }
            if (enteredToe) { --nEntered; enteredToe = false; toeEntryTime = -1.0; }
            if (reachedCollector) { --nCollector; reachedCollector = false; }
            if (collectedIn10m) {
                --nCollected10;
                if (impactBin >= 0 && impactBin < nRows && impactHits[impactBin] > 0) {
                    --impactHits[impactBin];
                }
                collectedIn10m = false;
            }
            reachedEnd = stopped = timedOut = false;
        } else {
            ++nSuccess;
            if (reachedEnd) ++nReachedEnd;
            if (stopped) ++nStopped;
            if (timedOut) ++nTimeout;

            sumFX += finalX; sumFX2 += finalX * finalX;
            sumFZ += finalZ; sumFZ2 += finalZ * finalZ;
            sumMaxV += maxSpeed; sumMaxV2 += maxSpeed * maxSpeed;
            sumPath += pathLength; sumPath2 += pathLength * pathLength;
            sumTime += t; sumTime2 += t * t;
        }

        char errorField[160];
        batchCsvText(errorField, sizeof(errorField), failed ? errorMsg : "ok");

        std::fprintf(fpTrials,
            "%d,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.4f,%.4f,%.4f,%.4f,"
            "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
            "%d,%d,%.4f,%.4f,%d,%d,%d,%d,%d,%s,%.4f,%.4f\n",
            trial, x0, groundZ0, v0, radius, area, sides, mu, e, cd, crr,
            finalX, finalZ, finalSpeed, maxSpeed, maxHeight, t,
            enteredToe ? 1 : 0, reachedCollector ? 1 : 0,
            collectorTime, collectorZ,
            collectedIn10m ? 1 : 0, reachedEnd ? 1 : 0,
            stopped ? 1 : 0, timedOut ? 1 : 0,
            failed ? 1 : 0, errorField, pathLength, toeEntryTime);

        if ((trial + 1) % 10 == 0 || trial + 1 == nTrials) {
            std::printf("  trial %d / %d\r", trial + 1, nTrials);
            std::fflush(stdout);
        }
    }
    std::printf("\n");

    for (int row = 0; row < nRows; ++row) {
        for (int col = 0; col < nCols; ++col) {
            int idx = row * nCols + col;
            double xmin = toeStartX + col * CELL_X;
            double xmax = xmin + CELL_X;
            double zmin = row * CELL_Z;
            double zmax = zmin + CELL_Z;
            if (xmax > endX) xmax = endX;
            if (zmax > TOE_HEIGHT) zmax = TOE_HEIGHT;
            double cx = 0.5 * (xmin + xmax);
            double cz = 0.5 * (zmin + zmax);
            double groundZ = hill.getHeight(cx);
            double pAll = (double)cellHits[idx] / (double)nTrials;
            double pEnt = (nEntered > 0)
                        ? (double)cellHits[idx] / (double)nEntered : 0.0;
            std::fprintf(fpGrid,
                "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.6f,%.6f\n",
                xmin, xmax, zmin, zmax, cx, cz, groundZ,
                cellHits[idx], pAll, pEnt);
        }
    }

    for (int col = 0; col < nCols; ++col) {
        double xmin = toeStartX + col * CELL_X;
        double xmax = xmin + CELL_X;
        if (xmax > endX) xmax = endX;
        double cx = 0.5 * (xmin + xmax);
        double groundZ = hill.getHeight(cx);
        double pAll = (double)colHits[col] / (double)nTrials;
        double pEnt = (nEntered > 0)
                    ? (double)colHits[col] / (double)nEntered : 0.0;
        std::fprintf(fpProfile, "%.3f,%.3f,%.3f,%.3f,%d,%.6f,%.6f\n",
                     xmin, xmax, cx, groundZ, colHits[col], pAll, pEnt);
    }

    for (int bin = 0; bin < nRows; ++bin) {
        double zmin = bin * CELL_Z;
        double zmax = zmin + CELL_Z;
        if (zmax > TOE_HEIGHT) zmax = TOE_HEIGHT;
        double cz = 0.5 * (zmin + zmax);
        double pAll = (double)impactHits[bin] / (double)nTrials;
        double pCond = (nCollected10 > 0)
                     ? (double)impactHits[bin] / (double)nCollected10 : 0.0;
        std::fprintf(fpImpact, "%.3f,%.3f,%.3f,%d,%.6f,%.6f\n",
                     zmin, zmax, cz, impactHits[bin], pAll, pCond);
    }

    std::fprintf(fpSummary,
        "n_trials,seed,toe_start_x,toe_height,"
        "n_entered_toe,p_entered_toe,"
        "n_reached_collector,p_reached_collector,"
        "n_collected_0_10m,p_collected_0_10m,"
        "n_reached_end,p_reached_end,"
        "n_stopped,p_stopped,n_timed_out,p_timed_out,"
        "n_failed,p_failed,n_success,"
        "mean_final_x,std_final_x,mean_final_z,std_final_z,"
        "mean_max_speed,std_max_speed,"
        "mean_path_length,std_path_length,"
        "mean_sim_time,std_sim_time\n");
    std::fprintf(fpSummary,
        "%d,%u,%.3f,%.3f,%d,%.6f,%d,%.6f,%d,%.6f,%d,%.6f,%d,%.6f,%d,%.6f,%d,%.6f,%d,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
        nTrials, seed, toeStartX, TOE_HEIGHT,
        nEntered, (double)nEntered / (double)nTrials,
        nCollector, (double)nCollector / (double)nTrials,
        nCollected10, (double)nCollected10 / (double)nTrials,
        nReachedEnd, (double)nReachedEnd / (double)nTrials,
        nStopped, (double)nStopped / (double)nTrials,
        nTimeout, (double)nTimeout / (double)nTrials,
        nFailed, (double)nFailed / (double)nTrials,
        nSuccess,
        batchMean(sumFX, nSuccess), batchStd(sumFX, sumFX2, nSuccess),
        batchMean(sumFZ, nSuccess), batchStd(sumFZ, sumFZ2, nSuccess),
        batchMean(sumMaxV, nSuccess), batchStd(sumMaxV, sumMaxV2, nSuccess),
        batchMean(sumPath, nSuccess), batchStd(sumPath, sumPath2, nSuccess),
        batchMean(sumTime, nSuccess), batchStd(sumTime, sumTime2, nSuccess));

    std::fclose(fpTrials);
    std::fclose(fpGrid);
    std::fclose(fpProfile);
    std::fclose(fpImpact);
    std::fclose(fpTraj);
    std::fclose(fpSummary);

    std::printf("Batch simulation finished.\n");
    std::printf("  trials              : %d\n", nTrials);
    std::printf("  seed                : %u\n", seed);
    std::printf("  toe region          : x in [%.2f, %.2f] m, z in [0, %.1f] m\n",
                toeStartX, endX, TOE_HEIGHT);
    std::printf("  entered toe         : %d (%.1f%%)\n",
                nEntered, 100.0 * nEntered / nTrials);
    std::printf("  reached collector   : %d (%.1f%%)\n",
                nCollector, 100.0 * nCollector / nTrials);
    std::printf("  collected in 0-10 m : %d (%.1f%%)\n",
                nCollected10, 100.0 * nCollected10 / nTrials);
    std::printf("  reached end         : %d (%.1f%%)\n",
                nReachedEnd, 100.0 * nReachedEnd / nTrials);
    std::printf("  stopped on slope    : %d (%.1f%%)\n",
                nStopped, 100.0 * nStopped / nTrials);
    std::printf("  timeout             : %d (%.1f%%)\n",
                nTimeout, 100.0 * nTimeout / nTrials);
    std::printf("  failed              : %d (%.1f%%)\n",
                nFailed, 100.0 * nFailed / nTrials);
    std::printf("  CSV files written to the current directory.\n");
    return 0;
}

#endif // BATCH_STATS_H
