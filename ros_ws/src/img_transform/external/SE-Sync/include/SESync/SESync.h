/** This file provides a convenient functional interface to the SESync
 * algorithm.
 *
 * Copyright (C) 2016 - 2018 by David M. Rosen (dmrosen@mit.edu)
 */

#pragma once

#include <vector>

#include <Eigen/Dense>

#include "SESync/RelativePoseMeasurement.h"
#include "SESync/SESyncProblem.h"
#include "SESync/SESync_types.h"

namespace SESync
{

    /** This struct contains the various parameters that control the SESync
     * algorithm */
    struct SESyncOpts
    {
        /// OPTIMIZATION STOPPING CRITERIA

        /** Stopping tolerance for the norm of the Riemannian gradient */
        Scalar grad_norm_tol = 1e-2;

        /** Stopping tolerance for the norm of the preconditioned Riemannian gradient
         */
        Scalar preconditioned_grad_norm_tol = 1e-4;

        /** Stopping criterion based upon the relative decrease in function value */
        Scalar rel_func_decrease_tol = 1e-7;

        /** Stopping criterion based upon the norm of an accepted update step */
        Scalar stepsize_tol = 1e-3;

        /** Maximum permitted number of (outer) iterations of the Riemannian
         * trust-region method when solving each instance of Problem 9 */
        size_t max_iterations = 1000;

        /** Maximum number of inner (truncated conjugate-gradient) iterations to
         * perform per out iteration */
        size_t max_tCG_iterations = 10000;

        /// These next two parameters define the stopping criteria for the truncated
        /// preconditioned conjugate-gradient solver running in the inner loop --
        /// they control the tradeoff between the quality of the returned
        /// trust-region update step (as a minimizer of the local quadratic model
        /// computed at each iterate) and the computational expense needed to generate
        /// that update step.  You probably don't need to modify these unless you
        /// really know what you're doing.

        /** Gradient tolerance for the truncated preconditioned conjugate gradient
         * solver: stop if ||g|| < kappa * ||g_0||.  This parameter should be in the
         * range (0,1). */
        Scalar STPCG_kappa = .1;

        /** Gradient tolerance based upon a fractional-power reduction in the norm of
         * the gradient: stop if ||g|| < ||kappa||^{1+ theta}.  This value should be
         * positive, and controls the asymptotic convergence rate of the
         * truncated-Newton trust-region solver: specifically, for theta > 0, the TNT
         * algorithm converges q-superlinearly with order (1+theta). */
        Scalar STPCG_theta = .5;

        /** Maximum elapsed computation time (in seconds) */
        double max_computation_time = std::numeric_limits<double>::max();

        /** An optional user-supplied function that can be used to instrument/monitor
         * the performance of the internal Riemannian truncated-Newton trust-region
         * optimization algorithm as it runs. */
        std::experimental::optional<SESyncTNTUserFunction> user_function;

        /// SE-SYNC PARAMETERS

        /** The specific formulation of the SE-Sync problem to solve */
        Formulation formulation = Formulation::Simplified;

        /** The initial level of the Riemannian Staircase */
        size_t r0 = 3;

        /** The maximum level of the Riemannian Staircase to explore */
        size_t rmax = 10;

        /** The maximum number of Lanczos iterations to admit for eigenvalue
         * computations */
        size_t max_eig_iterations = 10000;

        /** A numerical tolerance for acceptance of the minimum eigenvalue of Q -
         * Lambda(Y*) as numerically nonnegative; this should be a small positive
         * value e.g. 10^-4 */
        Scalar min_eig_num_tol = 1e-5;

        /** The number of working vectors to use in the minimum eigenvalue computation
         * (using the implicitly-restarted Arnoldi algorithm); must be in the range
         * [1, (#poses) * (#problem dimension) - 1] */
        size_t num_Lanczos_vectors = 40;

        /** Whether to use the Cholesky or QR factorization when computing the
         * orthogonal projection */
        ProjectionFactorization projection_factorization =
            ProjectionFactorization::Cholesky;

        /** The preconditioning strategy to use in the Riemannian trust-region
         * algorithm*/
        Preconditioner preconditioner = Preconditioner::RegularizedCholesky;

        /** Maximum admissible condition number for the regularized Cholesky
         * preconditioner */
        Scalar reg_Cholesky_precon_max_condition_number = 1e6;

        /** If no initial iterate Y0 is supplied, this boolean determines the
         * initialization strategy employed by SE-Sync: 'true' -> chordal, 'false' ->
         * random sampling */
        Initialization initialization = Initialization::Chordal;

        /** Whether to print output as the algorithm runs */
        bool verbose = false;

        /** If this value is true, the SE-Sync algorithm will log and return the
         * entire sequence of iterates generated by the Riemannian Staircase */
        bool log_iterates = false;

        /** The number of threads to use for parallelization (assuming that SE-Sync is
         * built using a compiler that supports OpenMP */
        size_t num_threads = 1;
    };

    /** These enumerations describe the termination status of the SE-Sync algorithm
     */
    enum SESyncStatus
    {
        /** The algorithm converged to a certified global optimum */
        GLOBAL_OPT,

        /** The algorithm converged to a saddle point, but the backtracking line
         * search was unable to escape it */
        SADDLE_POINT,

        /** The algorithm converged to a first-order critical point, but the
         * minimum-eigenvalue computation did not converge to sufficient precision to
         * enable its characterization */
        EIG_IMPRECISION,

        /** The algorithm exhausted the maximum number of iterations of the Riemannian
         * Staircase before finding an optimal solution */
        RS_ITER_LIMIT,

        /** The algorithm exhausted the allotted total computation time before finding
         * an optimal solution */
        ELAPSED_TIME
    };

    /** This struct contains the output of the SESync algorithm */
    struct SESyncResult
    {

        /** An estimate of a global minimizer Yopt of the rank-restricted dual
         * semidefinite relaxation Problem 9 in the SE-Sync tech report.  The
         * corresponding solution of Problem 7 is Z = Y^T Y */
        Matrix Yopt;

        /** The value of the objective F(Y^T Y) = F(Z) attained by the Yopt */
        Scalar SDPval;

        /** The norm of the Riemannian gradient at Yopt */
        Scalar gradnorm;

        /** The Lagrange multiplier matrix Lambda corresponding to Yopt, computed
         * according to eq. (119) in the SE-Sync tech report.  If Z = Y^T Y is an
         * exact solution for the dual semidefinite relaxation Problem 7, then Lambda
         * is the solution to the primal Lagrangian relaxation Problem 6. */
        SparseMatrix Lambda;

        /** The trace of Lambda; this is the value of Lambda under the objective of
            l) semidefinite relaxation Problem 6. */
        Scalar trace_Lambda;

        /** The duality gap between the estimates for the primal and dual solutions
         * Lambda and Z = Y^T Y of Problems 7 and 6, respectively; it is given by:
         *
         * SDP_duality_gap := F(Y^T Y) - tr(Lambda)
         *
         */
        Scalar SDP_duality_gap;

        /** The minimum eigenvalue of the matrix S - Lambda */
        Scalar lambda_min;

        /** The corresponding eigenvector of the minimum eigenvalue */
        Vector v_min;

        /** The value of the rounded solution xhat in SE(d)^n */
        Scalar Fxhat;

        /** The rounded solution xhat = [t | R] in SE(d)^n */
        Matrix xhat;

        /** Upper bound on the global suboptimality of the recovered pose estimates
         * xhat; this is equal to F(xhat) - tr(Lambda) */
        Scalar suboptimality_upper_bound;

        /** The total elapsed computation time for the SE-Sync algorithm */
        double total_computation_time;

        /** The elapsed computation time used to compute the initialization for the
         * Riemannian Staircase */
        double initialization_time;

        /** A vector containing the sequence of function values obtained during the
         * optimization at each level of the Riemannian Staircase */
        std::vector<std::vector<Scalar>> function_values;

        /** A vector containing the sequence of norms of the Riemannian gradients
         * obtained during the optimization at each level of the Riemannian Staircase
         */
        std::vector<std::vector<Scalar>> gradient_norms;

        /** A vector containing the sequence of (# Hessian-vector product operations)
         * carried out during the optimization at each level of the Riemannian
         * Staircase */
        std::vector<std::vector<size_t>> Hessian_vector_products;

        /** A vector containing the sequence of elapsed times in the optimization at
         * each level of the Riemannian Staircase at which the corresponding function
         * values and gradients were obtained */
        std::vector<std::vector<double>> elapsed_optimization_times;
        double elasped_optimization_time = 0;

        /** A vector containing the sequence of minimum eigenvalues of the certificate
         * matrix constructed from the critical point recovered from the
         * optimization at each level of the Riemannian Staircase */
        std::vector<Scalar> minimum_eigenvalues;

        /** A vector containing the number of matrix-vector multiplication operations
         * performed for the minimum-eigenvalue computation at each level of the
         * Riemannian Staircase */
        std::vector<size_t> minimum_eigenvalue_matrix_ops;

        /** A vector containing the elapsed time of the minimum eigenvalue computation
         * at each level of the Riemannian Staircase */
        std::vector<double> minimum_eigenvalue_computation_times;

        /** If SE-Sync was run with log_iterates = true, this will contain the
         * sequence of iterates generated by the truncated-Newton trust-region method
         * at each level of the Riemannian Staircase */
        std::vector<std::vector<Matrix>> iterates;

        /** The termination status of the SE-Sync algorithm */
        SESyncStatus status;
    };

    /** Given an SESyncProblem instance, this function performs synchronization */
    SESyncResult SESync(SESyncProblem &problem,
                        const SESyncOpts &options = SESyncOpts(),
                        const Matrix &Y0 = Matrix());
    
    /** Given an SESyncProblem instance, this function performs synchronization */
    void SESync(SESyncResult &SESyncResults, SESyncProblem &problem,
                const SESyncOpts &options = SESyncOpts(),
                const Matrix &Y0 = Matrix());

    /** Given a vector of relative pose measurements specifying a special Euclidean
     * synchronization problem, performs synchronization using the SESync algorithm
     */
    SESyncResult SESync(const measurements_t &measurements,
                        const SESyncOpts &options = SESyncOpts(),
                        const Matrix &Y0 = Matrix());

    /** Given a vector of relative pose measurements specifying a special Euclidean
     * synchronization problem, performs synchronization using the SESync algorithm
     */
    void SESync(SESyncResult &SESyncResults, 
                const measurements_t &measurements,
                const SESyncOpts &options = SESyncOpts(),
                const Matrix &Y0 = Matrix());                    

    /** Helper function: used in the Riemannian Staircase to escape from a saddle
     *  point.  Here:
     *
     * - problem is the specific special Euclidean synchronization problem we are
     *     attempting to solve
     * - Y is the critical point (saddle point) obtained at the current level of the
     *     Riemannian Staircase
     * - lambda_min is the (negative) minimum eigenvalue of the matrix Q - Lambda
     * - v_min is the eigenvector corresponding to lambda_min
     * - gradient_tolerance is a *lower bound* on the norm of the Riemannian
     *   gradient grad F(Yplus) in order to accept a candidate point Yplus as a
     *   valid solution
     *
     * Post-condition:  This function returns a Boolean value indicating whether it
     * was able to successfully escape from the saddle point, meaning it found a
     * point Yplus satisfying the following two criteria:
     *
     *  (1)  F(Yplus) < F(Y), and
     *  (2)  || grad F(Yplus) || > gradient_tolerance
     *
     * Condition (2) above is necessary to ensure that the optimization initialized
     * at the next level of the Riemannian Staircase does not immediately terminate
     * due to the gradient stopping tolerance being satisfied.
     *
     * Precondition: the relaxation rank r of 'problem' must be 1 greater than the
     * number of rows of Y (i.e., the relaxation rank of 'problem' must already be
     * set for the *next* level of the Riemannian Staircase when this function is
     * called.
     *
     * Postcondition: If this function returns true, then upon termination Yplus
     * contains the point at which to initialize the optimization at the next level
     * of the Riemannian Staircase
     */
    bool escape_saddle(const SESyncProblem &problem, const Matrix &Y,
                       Scalar lambda_min, const Vector &v_min,
                       Scalar gradient_tolerance,
                       Scalar preconditioned_gradient_tolerance, Matrix &Yplus);

} // namespace SESync
