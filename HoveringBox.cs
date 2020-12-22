using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class HoveringBox : MonoBehaviour
{
	[HideInInspector] public Vector2 input;
	bool paused;

	public Transform[] wheels = new Transform[4];
	RaycastHit[] hitInfo = new RaycastHit[4];
	public Rigidbody rb;
	public Transform centerOfMass;
	float horizontalAngularDrag;
	public float groundHorizontalAngularDrag;

	[Header("Hovering physics")]
	public float hoverHeight;
	public float groundGripHeight;
	public float originOffset;
	public float suspensionStrength;
	public float suspensionDamping;
	public float sphereCastRadius;
	public LayerMask ground;
	Vector3 surfaceNormal;
	Vector3 castDirection;

	[Header("Forward motion")]
	public float forwardAcceleration;
	public float maxSpeed;
	float forwardFriction;
	public float maxBackwardSpeed;
	float backwardAcceleration;
	public float slowingFactor;
	public float maxGroundAngle;
	float minGroundDotProduct;
	public float thresholdSpeed;
	[HideInInspector] public float forwardSpeed;

	[Header("Turning")]
	public float torqueMagnitude;
	public float maxTurnSpeed;
	float turningDrag;
	public float lateralFriction;
	public AnimationCurve slideFrictionCurve;
	float slideFriction;

	[Header("In Air")]
	public bool stabilizeInAir;
	Vector3 landingPosition;
	Vector3 landingNormal;
	float fallTime = 1f;
	float timeInAir;
	bool trajectoryFound;
	public float maxFlightTime;
	public float inAirHorizontalAngularDrag;
	public float alingmentForce;
	public float inAirVerticlalAngularDrag;

	public bool playerControlled;

    // Start is called before the first frame update
    void Start()
    {
		Cursor.lockState = CursorLockMode.Locked;
		Cursor.visible = false;
		rb.centerOfMass = centerOfMass.localPosition;
		forwardFriction = forwardAcceleration / maxSpeed;
		backwardAcceleration = maxBackwardSpeed * forwardFriction;
		turningDrag = torqueMagnitude / maxTurnSpeed;
		minGroundDotProduct = Mathf.Cos(maxGroundAngle * Mathf.Deg2Rad);
		castDirection = -transform.up;
		horizontalAngularDrag = groundHorizontalAngularDrag;
    }

	void Update() {
		// Control.
		if (playerControlled) {
			input = new Vector2(Input.GetAxis("Horizontal"), Input.GetAxis("Vertical"));
		}

		// Reset.
		if (Input.GetKeyDown(KeyCode.R)) {
			SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex);
		}

		// Pause.
		if (Input.GetKeyDown(KeyCode.P)) {
			if (!paused) {
				Time.timeScale = 0;
			} else {
				Time.timeScale = 1;
			}

			paused = !paused;
		}

		// Quit.
		if (Input.GetKeyDown(KeyCode.Escape))
		{
			Application.Quit();
		}

		// Test stability.
		if (Input.GetKeyDown(KeyCode.Space))
		{
			Vector2 randomVector2 = Random.insideUnitCircle;
			Vector3 rotationAxis = new Vector3(randomVector2.x, 0f, randomVector2.y);
			rb.AddTorque(rotationAxis * 100f, ForceMode.Impulse);
			rb.AddForce(-castDirection * 50f, ForceMode.Impulse);
			trajectoryFound = false;
		}
	}

	public bool UpsideDown => Vector3.Dot(Vector3.up, transform.up) < 0;
	public bool AllWheelsGrounded => hitInfo[0].collider != null && hitInfo[1].collider != null && hitInfo[2].collider != null && hitInfo[3].collider != null;
	public bool AllWheelsInAir => hitInfo[0].collider == null && hitInfo[1].collider == null && hitInfo[2].collider == null && hitInfo[3].collider == null;
	public bool PartiallyGrounded => !AllWheelsGrounded && !AllWheelsInAir;

    void FixedUpdate()
    {
		// Hovering physics.
		if (rb.velocity.magnitude < thresholdSpeed && AllWheelsGrounded) {
			castDirection = Vector3.down;
		} else {
			castDirection = UpsideDown ? transform.up : -transform.up;
		}

		RaycastHit hit;
		for (int i = 0; i < 4; i++) {
			Physics.SphereCast(wheels[i].position - castDirection * originOffset, sphereCastRadius, castDirection, out hit, groundGripHeight + originOffset);
			hitInfo[i] = hit;
		}

		if (AllWheelsGrounded) {
			for (int i = 0; i < 4; i++) {
				float displacement = hoverHeight - (hitInfo[i].distance - originOffset);
				float verticalSpeed = Vector3.Dot(-castDirection, rb.GetPointVelocity(wheels[i].position));
				float resistingForce = suspensionStrength * displacement - suspensionDamping * verticalSpeed;
				rb.AddForceAtPosition(resistingForce * -castDirection, wheels[i].position, ForceMode.Acceleration);
			}
			surfaceNormal = (hitInfo[0].normal + hitInfo[3].normal).normalized;

			trajectoryFound = false;
			horizontalAngularDrag = groundHorizontalAngularDrag;

			forwardSpeed = Vector3.Dot(rb.velocity, transform.forward);
			float sidewaysSpeed = Vector3.Dot(rb.velocity, transform.right);

			if (Mathf.Abs(sidewaysSpeed) + Mathf.Abs(forwardSpeed) < thresholdSpeed) {
				if (input.y == 0) {
					rb.AddForce(-rb.velocity * 0.05f, ForceMode.VelocityChange);
				}
			} else {
				// Forward friction.
				float inputCoefficient = Mathf.Abs(input.y) > Mathf.Epsilon ? 1f : slowingFactor;
				rb.AddForce(transform.forward * -forwardSpeed * forwardFriction * inputCoefficient, ForceMode.Acceleration);

				// Lateral friction.
				slideFriction = slideFrictionCurve.Evaluate(Mathf.Abs(sidewaysSpeed) / (Mathf.Abs(sidewaysSpeed) + Mathf.Abs(forwardSpeed)));
				rb.AddForce(-lateralFriction * slideFriction * sidewaysSpeed * transform.right, ForceMode.Acceleration);
			}

			// Forward acceleration.
			if (surfaceNormal.y >= minGroundDotProduct) {
				Vector3 forwardProjection = transform.forward - (Vector3.Dot(transform.forward, surfaceNormal)) * surfaceNormal;
				float acc = input.y > 0 ? forwardAcceleration : backwardAcceleration;
				rb.AddForce(input.y * acc * forwardProjection, ForceMode.Acceleration);
			}
		} else {
			horizontalAngularDrag = inAirHorizontalAngularDrag;

			// Recalculate trajectory.
			if (!trajectoryFound || PartiallyGrounded) {
				if (OnTakeoff != null) {
					OnTakeoff();
				}

				// Estimate landing point and find ground normal.
				fallTime = 0;
				Vector3 prevPos = transform.position + castDirection * groundGripHeight;
				Vector3 nextPos;
				Vector3 vel = rb.velocity;
				int maxNumSteps = Mathf.RoundToInt(maxFlightTime / Time.fixedDeltaTime);
				for (int i = 0; i < maxNumSteps; i++) {
					fallTime += Time.fixedDeltaTime;
					vel += Physics.gravity * Time.fixedDeltaTime;
					nextPos = prevPos + vel * Time.fixedDeltaTime;
					prevPos = nextPos;

					if (Physics.Raycast(prevPos, vel, out hit, vel.magnitude * Time.fixedDeltaTime, ground)) {
						landingPosition = hit.point;
						landingNormal = hit.normal;
						trajectoryFound = true;
						break;
					}
				}
			}

			// Aling vehicle with the landing normal.
			if (stabilizeInAir && trajectoryFound) {
				float angleFraction = Vector3.Angle(-castDirection, landingNormal) / 90f;
				Vector3 axisOfRotation = Vector3.Cross(-castDirection, landingNormal).normalized;
				rb.AddTorque(axisOfRotation * (alingmentForce / (fallTime + Mathf.Epsilon)) * angleFraction, ForceMode.Acceleration);
			}
		}

		// Turning.
		rb.AddTorque(input.x * -castDirection * torqueMagnitude, ForceMode.Acceleration);

		// Vertical angular drag.
		float verticalComponent = Vector3.Dot(rb.angularVelocity, transform.up);
		float verticalAngularDrag = input.x == 0 && AllWheelsInAir ? inAirVerticlalAngularDrag : turningDrag;
		rb.AddTorque(-verticalAngularDrag * verticalComponent * transform.up, ForceMode.Acceleration);

		// Horizontal angular drag.
		rb.AddTorque(-Vector3.Dot(rb.angularVelocity, transform.forward) * horizontalAngularDrag * transform.forward, ForceMode.Acceleration);
		rb.AddTorque(-Vector3.Dot(rb.angularVelocity, transform.right) * horizontalAngularDrag * transform.right, ForceMode.Acceleration);

		Debug.Log(rb.velocity.magnitude/maxSpeed);
    }

	public delegate void TakeoffDelegate();
	public static event TakeoffDelegate OnTakeoff;

	public void OnDrawGizmos() {
		Gizmos.color = Color.red;
		Gizmos.DrawSphere(landingPosition, 0.5f);
        for (int i = 0; i < 4; i++) {
			Transform wheel = wheels[i];
			// Gizmos.DrawSphere(wheel.position, 0.05f);
			if (hitInfo[i].collider != null) {
				Gizmos.color = Color.green;
				Gizmos.DrawLine(wheels[i].position, hitInfo[i].point);
				Gizmos.DrawWireSphere(wheels[i].position + castDirection * hitInfo[i].distance, 0.25f);
			} else {
				Gizmos.DrawLine(wheels[i].position, wheels[i].position + castDirection * (groundGripHeight + sphereCastRadius));
				Gizmos.DrawWireSphere(wheels[i].position + castDirection * groundGripHeight, 0.25f);
			}
		}
    }

	void OnCollisionEnter(Collision collision)
    {
		trajectoryFound = false;
    }

	void OnTriggerEnter(Collider collision)  {
		trajectoryFound = false;
	}
}
