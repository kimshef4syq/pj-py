#pragma once
#include <functional>
#include <RVS/Common/Timer.h>
#include <RVS/Common/Macros.h>
namespace RVS
{

RVS_CLASS_FORWARD(TerminateCondition);
/** @brief terminate condition used in planner */
class TerminateCondition
{
public:
    using TermFunction = typename std::function<bool()>;

    /** @brief Construct a new Terminate Condition object */
    TerminateCondition() = default;

    /** @brief construct TerminateCondition by passing function*/
    TerminateCondition(const TermFunction &fn_in) : m_fn(fn_in) {}

    /** @brief evaluate terminate condition
     *  @return true or false
     */
    virtual bool Eval() const { return m_fn(); }

protected:
    TermFunction m_fn;
};


/**
 * @brief Terminate condition tiggered by timer
 */
class TimedTerminateCondition : public TerminateCondition
{
public:
    /**
     * @brief Construct a new Timed Terminate Condition object
     * @param time_duration: time duration in sec
     */
    explicit TimedTerminateCondition(TimeDuration time_duration)
        : TerminateCondition(), m_duration(time_duration)
    {
        m_end_time = TimeStamp::Now() + time_duration;
    }

    /**
     * @brief Construct a new Timed Terminate Condition object
     * @param time_duration: time duration in sec
     */
    explicit TimedTerminateCondition(double time_duration)
        : TimedTerminateCondition(TimeDuration(time_duration))
    {
    }


    virtual bool Eval() const override
    {
        return (TimeStamp::Now() > m_end_time);
    }

protected:
    TimeStamp m_end_time;
    TimeDuration m_duration;
};

/**
 * @brief manully triggered terminate condition
 */
class TriggeredTerminateCondition : public TerminateCondition
{
public:
    /**
     * @brief Construct a new Triggered Terminate Condition object
     */
    TriggeredTerminateCondition() : TerminateCondition(), m_trigger_on(false) {}

    /**
     * @brief Set trigger on to terminate
     */
    void SetTriggerOn() { m_trigger_on = true; }

    virtual bool Eval() const override { return m_trigger_on; }

protected:
    bool m_trigger_on;
};


/**
 * @brief always terminate condition
 */
class AlwaysTerminateCondition : public TerminateCondition
{
public:
    /**
     * @brief Construct a new Always Terminate Condition object
     */
    AlwaysTerminateCondition() : TerminateCondition() {}

    virtual bool Eval() const override { return true; }
};


/**
 * @brief non terminate condition
 *
 */
class NonTerminateCondition : public TerminateCondition
{
public:
    /**
     * @brief Construct a new Non Terminate Condition object
     */
    NonTerminateCondition() : TerminateCondition() {}

    virtual bool Eval() const override { return false; }
};


/**
 * @brief `and` operation of terminate conditions, will be
 * triggered in condition `c1` and `c2`
 * @param c1: terminate condition
 * @param c2: terminate condition
 * @return TerminateConditionPtr
 */
TerminateConditionPtr AndTerminateCondition(const TerminateConditionPtr &c1,
                                            const TerminateConditionPtr &c2);


/**
 * @brief `or` operation of terminate conditions, will be
 * triggered in condition `c1` or `c2`
 * @param c1: terminate condition
 * @param c2: terminate condition
 * @return TerminateConditionPtr
 */
TerminateConditionPtr OrTerminateCondition(const TerminateConditionPtr &c1,
                                           const TerminateConditionPtr &c2);


}; // namespace RVS